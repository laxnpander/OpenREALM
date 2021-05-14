/**
* This file is part of OpenREALM.
*
* Copyright (C) 2020 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <realm_core/loguru.h>

#include <realm_stages/tileing.h>

using namespace realm;
using namespace stages;

Tileing::Tileing(const StageSettings::Ptr &stage_set, double rate)
    : StageBase("tileing", (*stage_set)["path_output"].toString(), rate, (*stage_set)["queue_size"].toInt(), bool((*stage_set)["log_to_file"].toInt())),
      m_utm_reference(nullptr),
      m_map_tiler(nullptr),
      m_tile_cache(nullptr),
      m_settings_save({})
{
  std::cout << "Stage [" << m_stage_name << "]: Created Stage with Settings: " << std::endl;
  stage_set->print();

  m_warper.setTargetEPSG(3857);
  m_warper.setNrofThreads(4);

  registerAsyncDataReadyFunctor([=]{ return !m_buffer.empty(); });
}

Tileing::~Tileing()
{
  if (m_tile_cache)
  {
    m_tile_cache->requestFinish();
    m_tile_cache->join();
  }
}

void Tileing::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateStatisticsIncoming();

  if (!frame->getSurfaceModel() || !frame->getOrthophoto())
  {
    LOG_F(INFO, "Input frame is missing data. Dropping!");
    updateStatisticsBadFrame();
    return;
  }
  std::unique_lock<std::mutex> lock(m_mutex_buffer);
  m_buffer.push_back(frame);

  // Ringbuffer implementation for buffer with no pose
  if (m_buffer.size() > m_queue_size)
  {
    updateStatisticsSkippedFrame();
    m_buffer.pop_front();
  }
  notify();
}

bool Tileing::process()
{
  bool has_processed = false;
  if (!m_buffer.empty() && m_map_tiler && m_tile_cache)
  {
    // Prepare timing
    long t;

    // Prepare output of incremental map update
    CvGridMap::Ptr map_update;

    Frame::Ptr frame = getNewFrame();

    LOG_F(INFO, "Processing frame #%u...", frame->getFrameId());

    if (m_utm_reference == nullptr)
      m_utm_reference = std::make_shared<UTMPose>(frame->getGnssUtm());

    //=======================================//
    //
    //   Step 1: Warp data to EPSG3857
    //
    //=======================================//

    t = getCurrentTimeMilliseconds();

    CvGridMap::Ptr orthophoto = frame->getOrthophoto();
    CvGridMap::Ptr surface_model = frame->getSurfaceModel();

    // Create the tiles, that will be visualized and therefore need multiple zoom levels
    CvGridMap::Ptr map = std::make_shared<CvGridMap>(orthophoto->roi(), orthophoto->resolution());
    map->add(*orthophoto, REALM_OVERWRITE_ALL, false);
    map->add(*surface_model, REALM_OVERWRITE_ALL, false);
    map->remove("num_observations"); // Currently not relevant for blending

    // Transform each layer of the CvGridMap separately to Web Mercator (EPSG:3857)
    std::vector<CvGridMap::Ptr> maps_3857;
    for (const auto &layer_name : map->getAllLayerNames())
    {
      maps_3857.emplace_back(m_warper.warpRaster(map->getSubmap({layer_name}), m_utm_reference->zone));
    }

    // Recombine to one single map
    auto map_3857 = std::make_shared<CvGridMap>(maps_3857[0]->roi(), maps_3857[0]->resolution());
    for (const auto &map_splitted : maps_3857)
    {
      // Each map only contains one layer, because we split them up before
      CvGridMap::Layer layer = map_splitted->getLayer(map_splitted->getAllLayerNames()[0]);
      map_3857->add(layer);
    }

    LOG_F(INFO, "Timing [Warping]: %lu ms", getCurrentTimeMilliseconds()-t);

    //=======================================//
    //
    //   Step 2: Tile the whole map on
    //           maximum zoom level
    //
    //=======================================//

    t = getCurrentTimeMilliseconds();

    std::map<int, MapTiler::TiledMap> tiled_map_max_zoom = m_map_tiler->createTiles(map_3857);

    LOG_F(INFO, "Timing [Tileing]: %lu ms", getCurrentTimeMilliseconds()-t);

    //=======================================//
    //
    //   Step 3: Blend the tiles on maximum
    //           zoom level and add to cache
    //
    //=======================================//

    t = getCurrentTimeMilliseconds();

    int zoom_level_max = tiled_map_max_zoom.begin()->first;

    std::vector<Tile::Ptr> tiles_current = tiled_map_max_zoom.begin()->second.tiles;
    std::vector<Tile::Ptr> tiles_blended;

    for (const auto &tile : tiles_current)
    {
      Tile::Ptr tile_cached = m_tile_cache->get(tile->x(), tile->y(), zoom_level_max);

      Tile::Ptr tile_blended;
      if (tile_cached)
      {
        tile_blended = blend(tile, tile_cached);
        tile_cached->unlock();
      }
      else
      {
        tile_blended = tile;
      }

      tiles_blended.push_back(tile_blended);
    }

    LOG_F(INFO, "Timing [Blending]: %lu ms", getCurrentTimeMilliseconds()-t);

    t = getCurrentTimeMilliseconds();
    m_tile_cache->add(zoom_level_max, tiles_blended, tiled_map_max_zoom.begin()->second.roi);
    LOG_F(INFO, "Timing [Cache Push]: %lu ms", getCurrentTimeMilliseconds()-t);

    //=======================================//
    //
    //   Step 4: Compute range of zoom levels
    //           and push to cache
    //
    //=======================================//

    t = getCurrentTimeMilliseconds();

    // To save computational load we remove layers, that we are not interested in displaying as a whole.
    // They can be required for the blending for example though, which is why we computed them on maximum resolution
    map_3857->remove("elevated");

    std::map<int, MapTiler::TiledMap> tiled_map_range = m_map_tiler->createTiles(map_3857, 11, zoom_level_max - 1);

    for (const auto& tiled_map : tiled_map_range)
    {
      int zoom_level = tiled_map.first;

      std::vector<Tile::Ptr> tiles_merged;
      for (const auto & tile : tiled_map.second.tiles)
      {
        Tile::Ptr tile_cached = m_tile_cache->get(tile->x(), tile->y(), zoom_level);

        Tile::Ptr tile_merged;
        if (tile_cached)
        {
          tile_merged = merge(tile, tile_cached);
          tile_cached->unlock();
        }
        else
        {
          tile_merged = tile;
        }

        tiles_merged.push_back(tile_merged);
      }

      m_tile_cache->add(zoom_level, tiles_merged, tiled_map.second.roi);
    }

    LOG_F(INFO, "Timing [Downscaling]: %lu ms", getCurrentTimeMilliseconds()-t);

    //=======================================//
    //
    //   Step 5: Publish & Save
    //
    //=======================================//

    // Publishings every iteration
    LOG_F(INFO, "Publishing...");

    t = getCurrentTimeMilliseconds();
    //publish(frame, _global_map, map_update, frame->getTimestamp());
    LOG_F(INFO, "Timing [Publish]: %lu ms", getCurrentTimeMilliseconds()-t);


    // Savings every iteration
    t = getCurrentTimeMilliseconds();
    saveIter(frame->getFrameId(), map_update);
    LOG_F(INFO, "Timing [Saving]: %lu ms", getCurrentTimeMilliseconds()-t);

    has_processed = true;
  }
  return has_processed;
}

Tile::Ptr Tileing::merge(const Tile::Ptr &t1, const Tile::Ptr &t2)
{
  if (t2->data()->empty())
    throw(std::runtime_error("Error: Tile data is empty. Likely a multi threading problem!"));

  t1->data()->add(*t2->data(), REALM_OVERWRITE_ZERO, false);

  return t1;
}

Tile::Ptr Tileing::blend(const Tile::Ptr &t1, const Tile::Ptr &t2)
{
  CvGridMap::Ptr& src = t2->data();
  CvGridMap::Ptr& dst = t1->data();

  // Cells in the elevation, which have no value are marked as NaN. Therefore v == v returns false for those.
  cv::Mat src_mask = ((*src)["elevation"] == (*src)["elevation"]);

  // Find all the cells, that are better in the destination tile
  // First get all elements in the destination tile, that are not elevated (have an elevation, but were not 3D reconstructed)
  cv::Mat dst_not_elevated;
  cv::bitwise_not((*dst)["elevated"], dst_not_elevated);

  // Now remove all cells from the source tile, that have a smaller elevation angle than the destination, except those
  // that are elevated where the destination is not.
  cv::Mat dst_mask = ((*src)["elevation_angle"] < (*dst)["elevation_angle"]) | ((*src)["elevated"] & dst_not_elevated);

  // Now remove them
  src_mask.setTo(0, dst_mask);

  (*src)["color_rgb"].copyTo((*dst)["color_rgb"], src_mask);
  (*src)["elevation"].copyTo((*dst)["elevation"], src_mask);
  (*src)["elevation_angle"].copyTo((*dst)["elevation_angle"], src_mask);

  return t1;
}

void Tileing::saveIter(uint32_t id, const CvGridMap::Ptr &map_update)
{
  /*if (_settings_save.save_valid)
    io::saveImage((*_global_map)["valid"], io::createFilename(_stage_path + "/valid/valid_", id, ".png"));
  if (_settings_save.save_ortho_rgb_all)
    io::saveImage((*_global_map)["color_rgb"], io::createFilename(_stage_path + "/ortho/ortho_", id, ".png"));
  if (_settings_save.save_elevation_all)
    io::saveImageColorMap((*_global_map)["elevation"], (*_global_map)["valid"], _stage_path + "/elevation/color_map", "elevation", id, io::ColormapType::ELEVATION);
  if (_settings_save.save_elevation_var_all)
    io::saveImageColorMap((*_global_map)["elevation_var"], (*_global_map)["valid"], _stage_path + "/variance", "variance", id,io::ColormapType::ELEVATION);
  if (_settings_save.save_elevation_obs_angle_all)
    io::saveImageColorMap((*_global_map)["elevation_angle"], (*_global_map)["valid"], _stage_path + "/obs_angle", "angle", id, io::ColormapType::ELEVATION);
  if (_settings_save.save_num_obs_all)
    io::saveImageColorMap((*_global_map)["num_observations"], (*_global_map)["valid"], _stage_path + "/nobs", "nobs", id, io::ColormapType::ELEVATION);
  if (_settings_save.save_ortho_gtiff_all && _gdal_writer != nullptr)
    _gdal_writer->requestSaveGeoTIFF(std::make_shared<CvGridMap>(_global_map->getSubmap({"color_rgb"})), _utm_reference->zone, _stage_path + "/ortho/ortho_iter.tif", true, _settings_save.split_gtiff_channels);*/

    //io::saveGeoTIFF(*map_update, "color_rgb", _utm_reference->zone, io::createFilename(_stage_path + "/ortho/ortho_", id, ".tif"));
}

void Tileing::saveAll()
{
  // 2D map output
//  if (_settings_save.save_ortho_rgb_one)
//    io::saveCvGridMapLayer(*_global_map, _utm_reference->zone, _utm_reference->band, "color_rgb", _stage_path + "/ortho/ortho.png");
//  if (_settings_save.save_elevation_one)
//    io::saveImageColorMap((*_global_map)["elevation"], (*_global_map)["valid"], _stage_path + "/elevation/color_map", "elevation", io::ColormapType::ELEVATION);
//  if (_settings_save.save_elevation_var_one)
//    io::saveImageColorMap((*_global_map)["elevation_var"], (*_global_map)["valid"], _stage_path + "/variance", "variance", io::ColormapType::ELEVATION);
//  if (_settings_save.save_elevation_obs_angle_one)
//    io::saveImageColorMap((*_global_map)["elevation_angle"], (*_global_map)["valid"], _stage_path + "/obs_angle", "angle", io::ColormapType::ELEVATION);
//  if (_settings_save.save_num_obs_one)
//    io::saveImageColorMap((*_global_map)["num_observations"], (*_global_map)["valid"], _stage_path + "/nobs", "nobs", io::ColormapType::ELEVATION);
//  if (_settings_save.save_num_obs_one)
//    io::saveGeoTIFF(_global_map->getSubmap({"num_observations"}), _utm_reference->zone, _stage_path + "/nobs/nobs.tif");
//  if (_settings_save.save_ortho_gtiff_one)
//    io::saveGeoTIFF(_global_map->getSubmap({"color_rgb"}), _utm_reference->zone, _stage_path + "/ortho/ortho.tif", true, _settings_save.split_gtiff_channels);
//  if (_settings_save.save_elevation_one)
//    io::saveGeoTIFF(_global_map->getSubmap({"elevation"}), _utm_reference->zone, _stage_path + "/elevation/gtiff/elevation.tif");
//
//  // 3D Point cloud output
//  if (_settings_save.save_dense_ply)
//  {
//    if (_global_map->exists("elevation_normal"))
//      io::saveElevationPointsToPLY(*_global_map, "elevation", "elevation_normal", "color_rgb", "valid", _stage_path + "/elevation/ply", "elevation");
//    else
//      io::saveElevationPointsToPLY(*_global_map, "elevation", "", "color_rgb", "valid", _stage_path + "/elevation/ply", "elevation");
//  }
//
//  // 3D Mesh output
//  if (_settings_save.save_elevation_mesh_one)
//  {
//    std::vector<cv::Point2i> vertex_ids = _mesher->buildMesh(*_global_map, "valid");
//    if (_global_map->exists("elevation_normal"))
//      io::saveElevationMeshToPLY(*_global_map, vertex_ids, "elevation", "elevation_normal", "color_rgb", "valid", _stage_path + "/elevation/mesh", "elevation");
//    else
//      io::saveElevationMeshToPLY(*_global_map, vertex_ids, "elevation", "", "color_rgb", "valid", _stage_path + "/elevation/mesh", "elevation");
//  }
}

void Tileing::reset()
{
  LOG_F(INFO, "Reseted!");
}

void Tileing::finishCallback()
{
  // Trigger savings
  saveAll();

}

Frame::Ptr Tileing::getNewFrame()
{
  std::unique_lock<std::mutex> lock(m_mutex_buffer);
  Frame::Ptr frame = m_buffer.front();
  m_buffer.pop_front();
  updateStatisticsProcessedFrame();
  return (std::move(frame));
}


void Tileing::initStageCallback()
{
  // Stage directory first
  if (!io::dirExists(m_stage_path))
    io::createDir(m_stage_path);

  // Then sub directories
  if (!io::dirExists(m_stage_path + "/tiles"))
    io::createDir(m_stage_path + "/tiles");

  // We can only create the map tiler,  when we have the final initialized stage path, which might be synchronized
  // across different devies. Consequently it is not created in the constructor but here.
  if (!m_map_tiler)
  {
    m_map_tiler = std::make_shared<MapTiler>(true);
    m_tile_cache = std::make_shared<TileCache>("tile_cache", 500, m_stage_path + "/tiles", false);
    m_tile_cache->start();
  }
}

void Tileing::printSettingsToLog()
{
  LOG_F(INFO, "### Stage process settings ###");
  //LOG_F(INFO, "- publish_mesh_nth_iter: %i", _publish_mesh_nth_iter);
}

void Tileing::publish(const Frame::Ptr &frame, const CvGridMap::Ptr &map, const CvGridMap::Ptr &update, uint64_t timestamp)
{
  // First update statistics about outgoing frame rate
  updateStatisticsOutgoing();

//  _transport_img((*_global_map)["color_rgb"], "output/rgb");
//  _transport_img(analysis::convertToColorMapFromCVC1((*_global_map)["elevation"],
//                                                      (*_global_map)["valid"],
//                                                      cv::COLORMAP_JET), "output/elevation");
//  _transport_cvgridmap(update->getSubmap({"color_rgb"}), _utm_reference->zone, _utm_reference->band,
//                       "output/update/ortho");
//  //_transport_cvgridmap(update->getSubmap({"elevation", "valid"}), _utm_reference->zone, _utm_reference->band, "output/update/elevation");
//
//  if (_publish_mesh_every_nth_kf > 0 && _publish_mesh_every_nth_kf == _publish_mesh_nth_iter)
//  {
//    std::vector <Face> faces = createMeshFaces(map);
//    std::thread t(_transport_mesh, faces, "output/mesh");
//    t.detach();
//    _publish_mesh_nth_iter = 0;
//  } else if (_publish_mesh_every_nth_kf > 0)
//  {
//    _publish_mesh_nth_iter++;
//  }
}

uint32_t Tileing::getQueueDepth()
{
  return m_buffer.size();
}