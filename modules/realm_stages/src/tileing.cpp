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
#include <cstdio>
#include <memory>
#include <utility>

#include <realm_core/loguru.h>
#include <realm_stages/tileing.h>
#include <realm_io/cv_import.h>
#include <realm_io/cv_export.h>

using namespace realm;
using namespace stages;

Tileing::Tileing(const StageSettings::Ptr &stage_set, double rate)
    : StageBase("tileing", (*stage_set)["path_output"].toString(), rate, (*stage_set)["queue_size"].toInt(), bool((*stage_set)["log_to_file"].toInt())),
      m_generate_tms_tiles((*stage_set)["tms_tiles"].toInt() > 0),
      m_min_tile_zoom((*stage_set)["min_zoom"].toInt()),
      m_max_tile_zoom((*stage_set)["max_zoom"].toInt()),
      m_delete_cache_on_init((*stage_set)["delete_cache_on_init"].toInt() > 0),
      m_load_cache_on_init((*stage_set)["load_cache_on_init"].toInt() > 0),
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

    // Get the image to add to the tile cache
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

    std::map<int, MapTiler::TiledMap> tiled_map_max_zoom = m_map_tiler->createTiles(map_3857, m_max_tile_zoom, m_max_tile_zoom);

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

    std::map<int, MapTiler::TiledMap> tiled_map_range = m_map_tiler->createTiles(map_3857, m_min_tile_zoom, zoom_level_max - 1);
    std::map<int, cv::Rect2i> tiles_merged_roi;

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
          // Currently merge appears to be having issues properly combining tiles, leading to
          // odd artifacts.  Merge is slightly more intensive, but works cleaner.  I am not yet sure why merge
          // is having issues.
          //tile_merged = merge(tile, tile_cached);
          tile_merged = blend(tile, tile_cached);
          tile_cached->unlock();
        }
        else
        {
          tile_merged = tile;
        }

        tiles_merged.push_back(tile_merged);
      }

      m_tile_cache->add(zoom_level, tiles_merged, tiled_map.second.roi);
      tiles_merged_roi[zoom_level] = tiled_map.second.roi;
    }

    LOG_F(INFO, "Timing [Downscaling]: %lu ms", getCurrentTimeMilliseconds()-t);

    //=======================================//
    //
    //   Step 5: Publish & Save
    //
    //=======================================//

    // Publishings every iteration, most publishing is done by separate IO thread when map is updated
    LOG_F(INFO, "Publishing...");
    t = getCurrentTimeMilliseconds();
    publish(frame, *m_tile_cache.get(), tiles_merged_roi, frame->getTimestamp());
    LOG_F(INFO, "Timing [Publish]: %lu ms", getCurrentTimeMilliseconds()-t);

    // Savings every iteration
    t = getCurrentTimeMilliseconds();
    //saveIter(frame->getFrameId(), map_update);
    LOG_F(INFO, "Timing [Saving]: %lu ms", getCurrentTimeMilliseconds()-t);

    has_processed = true;
  }
  return has_processed;
}

void Tileing::deleteCache() {
  if (m_tile_cache)
  {
    m_tile_cache->deleteCache();
  }
}

void Tileing::deleteCache(std::string layer) {
  if (m_tile_cache) {
    m_tile_cache->deleteCache(layer);
  }
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

  // There are apparently a number of issues with NaN comparisons breaking in various ways.  See:
  // https://github.com/opencv/opencv/issues/16465
  // To avoid these, use patchNaNs before using boolean comparisons
  cv::patchNaNs((*dst)["elevation_angle"],0);
  cv::Mat dst_mask = ((*src)["elevation_angle"] > (*dst)["elevation_angle"]);// | ((*src)["elevated"] & dst_not_elevated));

  (*src)["color_rgb"].copyTo((*dst)["color_rgb"], dst_mask);
  (*src)["elevation"].copyTo((*dst)["elevation"], dst_mask);
  (*src)["elevation_angle"].copyTo((*dst)["elevation_angle"], dst_mask);

  return t1;
}

void Tileing::saveIter(uint32_t id, const CvGridMap::Ptr &map_update)
{
   // Not really a good save iter, though we could save png representations of the tile cache?
}

void Tileing::saveAll()
{
  // Possibly merge tiles with gdal CoG if save option is set?
  // Easier CoG support would require GDAL 3.2.1, or custom calls
}

void Tileing::reset()
{
  LOG_F(INFO, "Reseted!");
}

void Tileing::finishCallback()
{
  if (m_tile_cache)
  {
    m_tile_cache->requestFinish();
    m_tile_cache->join();
  }

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

void Tileing::initStagePath(std::string stage_path) {
  StageBase::initStagePath(stage_path);
}
void Tileing::initStagePath(std::string stage_path, std::string cache_path) {
  // Set our cache path first before calling down to the stage initialization
  // The stage initialization callback will create directories and start the cache up.
  m_cache_path = cache_path;
  initStagePath(stage_path);
}

void Tileing::initStageCallback()
{
  if (m_cache_path.empty()) {
    m_cache_path = m_stage_path + "/tiles";
  }

  // Stage directory first
  if (!io::dirExists(m_stage_path))
    io::createDir(m_stage_path);

  // Initialize cache path
  if (!io::dirExists(m_cache_path))
    io::createDir(m_cache_path);

  // We can only create the map tiler,  when we have the final initialized stage path, which might be synchronized
  // across different devies. Consequently it is not created in the constructor but here.
  if (!m_map_tiler)
  {
    m_map_tiler = std::make_shared<MapTiler>(true, m_generate_tms_tiles);
    m_tile_cache = std::make_unique<TileCache>(this, 500, m_cache_path, true);

    // If both delete and load are selected, delete first, which will override the load
    if (m_delete_cache_on_init) {
      deleteCache();
    }
    if (m_load_cache_on_init) {
      m_tile_cache->loadDiskCache();
    }

    m_tile_cache->start();
  }
}

void Tileing::printSettingsToLog()
{
  LOG_F(INFO, "### Stage process settings ###");
  //LOG_F(INFO, "- publish_mesh_nth_iter: %i", _publish_mesh_nth_iter);
}

void Tileing::publish(const Frame::Ptr &frame, TileCache &cache, std::map<int, cv::Rect2i> updated_tiles, uint64_t timestamp) {
  // First update statistics about outgoing frame rate
  updateStatisticsOutgoing();

  // Right now we only update when we write tiles to the drive.  Optionally, we could publish changed tiles here, but since
  // we don't have anything that consumes them, we skip this step
}

uint32_t Tileing::getQueueDepth()
{
  return m_buffer.size();
}



TileCache::TileCache(Tileing *tiling_stage, double sleep_time, std::string output_directory, bool verbose)
    : WorkerThreadBase("tile_cache_io", sleep_time, verbose),
      m_dir_toplevel(std::move(output_directory)),
      m_has_init_directories(false),
      m_do_update(false),
      m_tiling_stage(tiling_stage)
{
  m_data_ready_functor = [=]{ return (m_do_update || isFinishRequested()); };
}

TileCache::~TileCache()
{
  flushAll();
  for (auto it = m_cache_bounds.begin(); it != m_cache_bounds.end(); it++) {
    LOG_F(INFO, "Final cache bounds for zoom %d : X %d - %d : Y %d - %d", it->first,
          it->second.x, it->second.x + it->second.width - 1,
          it->second.y, it->second.y + it->second.height - 1);
  }
}

void TileCache::setOutputFolder(const std::string &dir)
{
  std::lock_guard<std::mutex> lock(m_mutex_settings);
  m_dir_toplevel = dir;
}

std::string TileCache::getCachePath(const std::string &layer)
{
  std::string filename = m_dir_toplevel + "/"+ layer + "/";
  return filename;
}

bool TileCache::process()
{
  bool has_processed = false;

  if (m_mutex_do_update.try_lock())
  {
    long t;

    // Give update lock free as fast as possible, so we won't block other threads from adding data
    bool do_update = m_do_update;
    m_do_update = false;
    m_mutex_do_update.unlock();

    // Calculate the region where tiles were updated
    std::map<int, cv::Rect2i> write_region;

    if (do_update)
    {
      int n_tiles_written = 0;

      t = getCurrentTimeMilliseconds();

      for (auto &cached_elements_zoom : m_cache)
      {
        // Find our prediction region, default to a zero area prediction if it doesn't exist
        cv::Rect2i roi_prediction(0,0,0,0);
        if (m_roi_prediction.find(cached_elements_zoom.first) != m_roi_prediction.end()) {
          roi_prediction = m_roi_prediction.at(cached_elements_zoom.first);
        }

        for (auto &cached_elements_column : cached_elements_zoom.second)
        {
          for (auto &cached_elements : cached_elements_column.second)
          {
            std::lock_guard<std::mutex> lock(cached_elements.second->mutex);
            cached_elements.second->tile->lock();

            if (!cached_elements.second->was_written)
            {
              n_tiles_written++;
              write(cached_elements.second);

              // Update our roi containing written tiles
              auto write_roi = write_region.find(cached_elements_zoom.first);
              if (write_roi != write_region.end()) {
                write_roi->second |= cv::Rect2i(cached_elements.second->tile->x(), cached_elements.second->tile->y(), 1, 1);
              } else {
                write_region[cached_elements_zoom.first] = cv::Rect2i(cached_elements.second->tile->x(), cached_elements.second->tile->y(), 1, 1);
              }
            }

            if (isCached(cached_elements.second))
            {
              int tx = cached_elements.second->tile->x();
              int ty = cached_elements.second->tile->y();
              if (tx < roi_prediction.x || tx > roi_prediction.x + roi_prediction.width
                  || ty < roi_prediction.y || ty > roi_prediction.y + roi_prediction.height)
              {
                flush(cached_elements.second);
              }
            }
            cached_elements.second->tile->unlock();
          }
        }
      }

      if (n_tiles_written > 0) {
        // TODO: Update cache here instead of main, so we write when file system updates have occurred
        for (auto it = write_region.begin(); it != write_region.end(); it++) {
          LOG_IF_F(INFO, m_verbose, "Cache File Update for zoom %d : X %d - %d : Y %d - %d", it->first,
                it->second.x, it->second.x + it->second.width - 1,
                it->second.y, it->second.y + it->second.height - 1);
        }

        // Publish update files by zoom and region
        LOG_F(INFO, "Publishing...");
        t = getCurrentTimeMilliseconds();
        publishWrittenTiles(write_region, n_tiles_written);
        LOG_F(INFO, "Timing [Publish]: %lu ms", getCurrentTimeMilliseconds()-t);

      }
      LOG_IF_F(INFO, m_verbose, "Tiles written: %i", n_tiles_written);
      LOG_IF_F(INFO, m_verbose, "Timing [Cache Flush]: %lu ms", getCurrentTimeMilliseconds() - t);

      has_processed = true;
    }
  }
  return has_processed;
}

void TileCache::reset()
{
  m_cache.clear();
  m_cache_bounds.clear();
}

void TileCache::add(int zoom_level, const std::vector<Tile::Ptr> &tiles, const cv::Rect2i &roi_idx)
{
  std::lock_guard<std::mutex> lock(m_mutex_cache);

  // Assuming all tiles are based on the same data, therefore have the same number of layers and layer names
  std::vector<std::string> layer_names = tiles[0]->data()->getAllLayerNames();

  std::vector<LayerMetaData> layer_meta;
  for (const auto &layer_name : layer_names)
  {
    // Saving the name and the type of the layer into the meta data
    CvGridMap::Layer layer = tiles[0]->data()->getLayer(layer_name);
    layer_meta.emplace_back(LayerMetaData{layer_name, layer.data.type(), layer.interpolation});
  }

  if (!m_has_init_directories)
  {
    createDirectories(m_dir_toplevel + "/", layer_names, "");
    m_has_init_directories = true;
  }

  auto it_zoom = m_cache.find(zoom_level);

  long timestamp = getCurrentTimeMilliseconds();

  long t = getCurrentTimeMilliseconds();

  // Cache for this zoom level already exists
  if (it_zoom != m_cache.end())
  {
    for (const auto &t : tiles)
    {
      // Here we find a tile grid for a specific zoom level and add the new tiles to it.
      // Important: Tiles that already exist will be overwritten!
      t->lock();
      auto it_tile_x = it_zoom->second.find(t->x());
      if (it_tile_x == it_zoom->second.end())
      {
        // Zoom level exists, but tile column is
        createDirectories(m_dir_toplevel + "/", layer_names, "/" + std::to_string(zoom_level) + "/" + std::to_string(t->x()));
        it_zoom->second[t->x()][t->y()].reset(new CacheElement{timestamp, layer_meta, t, false});
      }
      else
      {
        auto it_tile_xy = it_tile_x->second.find(t->y());
        if (it_tile_xy == it_tile_x->second.end())
        {
          // Zoom level and column was found, but tile did not yet exist
          it_tile_x->second[t->y()].reset(new CacheElement{timestamp, layer_meta, t, false});
        }
        else
        {
          // Existing tile was found inside zoom level and column
          it_tile_xy->second->mutex.lock(); // note: mutex goes out of scope after this operation, no unlock needed.
          it_tile_xy->second.reset(new CacheElement{timestamp, layer_meta, t, false});
        }
      }
      t->unlock();
    }
  }
    // Cache for this zoom level does not yet exist
  else
  {
    createDirectories(m_dir_toplevel + "/", layer_names, "/" + std::to_string(zoom_level));

    CacheElementGrid tile_grid;
    for (const auto &t : tiles)
    {
      // By assigning a new grid of tiles to the zoom level we overwrite all existing data. But in this case there was
      // no prior data found for the specific zoom level.
      t->lock();
      auto it_tile_x = it_zoom->second.find(t->x());
      if (it_tile_x == it_zoom->second.end())
        createDirectories(m_dir_toplevel + "/", layer_names, "/" + std::to_string(zoom_level) + "/" + std::to_string(t->x()));

      tile_grid[t->x()][t->y()].reset(new CacheElement{timestamp, layer_meta, t, false});
      t->unlock();
    }
    m_cache[zoom_level] = tile_grid;
  }

  LOG_IF_F(INFO, m_verbose, "Timing [Cache Push]: %lu ms", getCurrentTimeMilliseconds() - t);

  // Finally, update the bounds to take into account the newly added tile
  auto bounds_iter = m_cache_bounds.find(zoom_level);
  if(bounds_iter != m_cache_bounds.end()) {
    bounds_iter->second |= roi_idx;
  } else {
    m_cache_bounds[zoom_level] = roi_idx;
  }

  updatePrediction(zoom_level, roi_idx);

  std::lock_guard<std::mutex> lock1(m_mutex_do_update);
  m_do_update = true;
  notify();
}

Tile::Ptr TileCache::get(int tx, int ty, int zoom_level)
{
  auto it_zoom = m_cache.find(zoom_level);
  if (it_zoom == m_cache.end())
  {
    return nullptr;
  }

  auto it_tile_x = it_zoom->second.find(tx);
  if (it_tile_x == it_zoom->second.end())
  {
    return nullptr;
  }

  auto it_tile_xy = it_tile_x->second.find(ty);
  if (it_tile_xy == it_tile_x->second.end())
  {
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(it_tile_xy->second->mutex);

  // Warning: We lock the tile now and return it to the calling thread locked. Therefore the responsibility to unlock
  // it is on the calling thread!
  it_tile_xy->second->tile->lock();
  if (!isCached(it_tile_xy->second))
  {
    load(it_tile_xy->second);
  }
  return it_tile_xy->second->tile;
}

std::map<int, cv::Rect2i> TileCache::getBounds() const
{
  return m_cache_bounds;
}

void TileCache::publishWrittenTiles(std::map<int, cv::Rect2i> &update_region, int num_tiles) {
  LOG_IF_F(INFO, m_verbose, "Publishing %d tiles...", num_tiles);
  m_tiling_stage->m_transport_tiling(getCachePath("rgb_color"), "png", update_region, "output/update/rgb_color");
  m_tiling_stage->m_transport_tiling(getCachePath("rgb_color"), "png", getBounds(), "output/full/rgb_color");
}

void TileCache::flushAll()
{
  int n_tiles_written = 0;

  // Calculate the region where tiles were updated
  std::map<int, cv::Rect2i> write_region;

  LOG_IF_F(INFO, m_verbose, "Flushing all tiles...");

  long t = getCurrentTimeMilliseconds();

  for (auto &zoom_levels : m_cache)
    for (auto &cache_column : zoom_levels.second)
      for (auto &cache_element : cache_column.second)
      {
        std::lock_guard<std::mutex> lock(cache_element.second->mutex);
        cache_element.second->tile->lock();
        if (!cache_element.second->was_written)
        {
          write(cache_element.second);
          n_tiles_written++;

          // Update our roi containing written tiles
          auto write_roi = write_region.find(zoom_levels.first);
          if (write_roi != write_region.end()) {
            write_roi->second |= cv::Rect2i(cache_element.second->tile->x(), cache_element.second->tile->y(), 1, 1);
          } else {
            write_region[zoom_levels.first] = cv::Rect2i(cache_element.second->tile->x(), cache_element.second->tile->y(), 1, 1);
          }
        }

        auto layers = cache_element.second->tile->data()->getAllLayerNames();
        for (auto layer : layers) {
          cache_element.second->tile->data()->remove(layer);
        }
        cache_element.second->tile->unlock();
      }

  if (n_tiles_written > 0) {
    // TODO: Update cache here instead of main, so we write when file system updates have occurred
    for (auto it = write_region.begin(); it != write_region.end(); it++) {
      LOG_IF_F(INFO, m_verbose, "Flushall File Update for zoom %d : X %d - %d : Y %d - %d", it->first,
            it->second.x, it->second.x + it->second.width - 1,
            it->second.y, it->second.y + it->second.height - 1);
    }

    // Publish update files by zoom and region
    LOG_F(INFO, "Publishing...");
    t = getCurrentTimeMilliseconds();
    publishWrittenTiles(write_region, n_tiles_written);
    LOG_F(INFO, "Timing [Publish]: %lu ms", getCurrentTimeMilliseconds()-t);
    publishWrittenTiles(write_region, n_tiles_written);
  }
}

void TileCache::loadAll()
{
  for (auto &zoom_levels : m_cache)
    for (auto &cache_column : zoom_levels.second)
      for (auto &cache_element : cache_column.second)
      {
        std::lock_guard<std::mutex> lock(cache_element.second->mutex);
        cache_element.second->tile->lock();
        if (!isCached(cache_element.second))
          load(cache_element.second);
        cache_element.second->tile->unlock();
      }
}


void TileCache::loadDiskCache()
{
  LOG_F(INFO, "Attempting to load pre-existing map from cache...");

  // Read which folders are present.  Ensure all folders required to load the cache are there
  if (!(boost::filesystem::exists(m_dir_toplevel + "/color_rgb") &&
        boost::filesystem::exists(m_dir_toplevel + "/elevation_angle") &&
        boost::filesystem::exists(m_dir_toplevel + "/elevation") &&
        boost::filesystem::exists(m_dir_toplevel + "/elevated"))) {
    LOG_F(WARNING, "One or more items required to load cache does NOT exist.  Creating new cache...");
    return;
  }

  int element_count = 0;

  // If all major folders are present, load all items that are on disk into, using RGB as a reference
  if (boost::filesystem::is_directory(m_dir_toplevel + "/color_rgb")) {

    // Sort in reverse order, so we add higher zoom levels first
    auto cache_files = io::getFileList(m_dir_toplevel + "/color_rgb", ".png", std::greater<>());

    for (auto& file : cache_files) {

      // Parse zoom, x, any y data from the path
      auto path = boost::filesystem::path(file);
      int z = std::stoi(path.parent_path().parent_path().filename().string());
      int x = std::stoi(path.parent_path().filename().string());
      int y = std::stoi(path.filename().stem().string());

      // Figure out the tile bounds to load
      cv::Rect2i data_roi(0, 0, 255, 255);
      double resolution = 1.0;

      // NOTE:
      // Currently, we don't store any way to tell the interpolation method used.  The code right now defaults to INTER_LINEAR, so we can assume that for now.
      // Longer term, it may make sense to write these settings out to the base folder of each category and load it again.
      std::vector<LayerMetaData> layers;

      // At a minimum, we should have color_rgb, elevation_angle, elevation.  Elevated is optional
      bool has_required_layers = true;

      // RGB Layer Cache - Should always exist.  To load the others
      layers.push_back(LayerMetaData{"color_rgb", CV_8UC4, cv::InterpolationFlags::INTER_LINEAR});

      // Elevation Angle Cache
      if (boost::filesystem::exists(m_dir_toplevel + "/elevation_angle/" + std::to_string(z) + "/" + std::to_string(x) + "/" + std::to_string(y) + ".bin")) {
        layers.push_back(LayerMetaData{"elevation_angle", CV_32FC1, cv::InterpolationFlags::INTER_LINEAR});
      } else {
        LOG_F(WARNING, "Unable to find required elevation_angle layer for z/x/y of %d / %d / %d.", z, x, y);
        has_required_layers = false;
      }

      // Elevation Cache
      if (boost::filesystem::exists(m_dir_toplevel + "/elevation/" + std::to_string(z) + "/" + std::to_string(x) + "/" + std::to_string(y) + ".bin")) {
        layers.push_back(LayerMetaData{"elevation", CV_32FC1, cv::InterpolationFlags::INTER_LINEAR});
      } else {
        LOG_F(WARNING, "Unable to find required elevation layer for z/x/y of %d / %d / %d.", z, x, y);
        has_required_layers = false;
      }

      // Elevated Cache (Only exists for highest zoom)
      if (boost::filesystem::exists(m_dir_toplevel + "/elevated/" + std::to_string(z) + "/" + std::to_string(x) + "/" + std::to_string(y) + ".png")) {
        layers.push_back(LayerMetaData{"elevated", CV_8UC1, cv::InterpolationFlags::INTER_LINEAR});
      }

      if (has_required_layers) {
        // Check if zoom already exists
        auto it_zoom = m_cache.find(z);
        if (it_zoom != m_cache.end()) {
          // Zoom exists

          // Check if x map already exists
          auto it_x = it_zoom->second.find(x);
          if (it_x != it_zoom->second.end()) {
            // X exists
            it_x->second[y].reset(new CacheElement{getCurrentTimeMilliseconds(), layers, Tile::Ptr(new Tile(z,x,y, CvGridMap(data_roi, 1.0), false)), true});
          } else {
            // X doesn't exist
            CacheElementItem x_entry;
            x_entry[y].reset(new CacheElement{getCurrentTimeMilliseconds(), layers, Tile::Ptr(new Tile(z,x,y, CvGridMap(data_roi, 1.0), false)), true});
            m_cache[z][x] = x_entry;
          }
        } else {
          // Zoom doesn't exist
          // Add to cache, but don't load any of the tiles
          CacheElementGrid tile_grid;
          tile_grid[x][y].reset(new CacheElement{getCurrentTimeMilliseconds(), layers, Tile::Ptr(new Tile(z,x,y, CvGridMap(data_roi, 1.0), false)), true});
          m_cache[z] = tile_grid;
        }

        element_count++;
      } else {
        LOG_F(WARNING, "Unable to find all layers for z/x/y of %d / %d / %d, skipping add to cache!", z, x, y);
      }
    }
  }

  LOG_F(INFO, "Loaded %d existing tiles from cache.", element_count);

}

void TileCache::deleteCache()
{
  // Remove all cache items
  flushAll();
  m_has_init_directories = false;
  auto files = io::getFileList(m_dir_toplevel, "");
  for (auto & file : files) {
    if (!file.empty()) io::removeFileOrDirectory(file);
  }
}

void TileCache::deleteCache(std::string layer)
{
  // Attempt to remove the specific layer name
  flushAll();
  m_has_init_directories = false;
  io::removeFileOrDirectory(m_dir_toplevel + "/" + layer);
}

void TileCache::load(const CacheElement::Ptr &element)
{
  for (const auto &meta : element->layer_meta)
  {
    std::string filename = m_dir_toplevel + "/"
                           + meta.name + "/"
                           + std::to_string(element->tile->zoom_level()) + "/"
                           + std::to_string(element->tile->x()) + "/"
                           + std::to_string(element->tile->y());

    int type = meta.type & CV_MAT_DEPTH_MASK;

    switch(type)
    {
      case CV_8U:
        filename += ".png";
        break;
      case CV_16U:
        filename += ".bin";
        break;
      case CV_32F:
        filename += ".bin";
        break;
      case CV_64F:
        filename += ".bin";
        break;
      default:
        throw(std::invalid_argument("Error reading tile: data type unknown!"));
    }

    if (io::fileExists(filename))
    {
      cv::Mat data = io::loadImage(filename);

      element->tile->data()->add(meta.name, data, meta.interpolation_flag);

      LOG_IF_F(INFO, m_verbose, "Read tile from disk: %s", filename.c_str());
    }
    else
    {
      LOG_IF_F(WARNING, m_verbose, "Failed reading tile from disk: %s", filename.c_str());
      throw(std::invalid_argument("Error loading tile."));
    }
  }
}

void TileCache::write(const CacheElement::Ptr &element)
{
  for (const auto &meta : element->layer_meta)
  {
    cv::Mat data = element->tile->data()->get(meta.name);

    std::string filename = m_dir_toplevel + "/"
                           + meta.name + "/"
                           + std::to_string(element->tile->zoom_level()) + "/"
                           + std::to_string(element->tile->x()) + "/"
                           + std::to_string(element->tile->y());

    int type = data.type() & CV_MAT_DEPTH_MASK;

    switch(type)
    {
      case CV_8U:
        filename += ".png";
        break;
      case CV_16U:
        filename += ".bin";
        break;
      case CV_32F:
        filename += ".bin";
        break;
      case CV_64F:
        filename += ".bin";
        break;
      default:
        throw(std::invalid_argument("Error writing tile: data type unknown!"));
    }

    io::saveImage(data, filename);

    element->was_written = true;
  }
}

void TileCache::flush(const CacheElement::Ptr &element)
{
  if (!element->was_written)
    write(element);

  for (const auto &meta : element->layer_meta)
  {
    element->tile->data()->remove(meta.name);
  }

  LOG_IF_F(INFO, m_verbose, "Flushed tile (%i, %i, %i) [zoom, x, y]", element->tile->zoom_level(), element->tile->x(), element->tile->y());
}

bool TileCache::isCached(const CacheElement::Ptr &element) const
{
  return !(element->tile->data()->empty());
}

size_t TileCache::estimateByteSize(const Tile::Ptr &tile) const
{
  tile->lock();
  //size_t bytes = tile->data().total() * tile->data().elemSize();
  tile->unlock();

  //return bytes;
  return 0;
}

void TileCache::updatePrediction(int zoom_level, const cv::Rect2i &roi_current)
{
  std::lock_guard<std::mutex> lock(m_mutex_roi_prev_request);
  std::lock_guard<std::mutex> lock1(m_mutex_roi_prediction);

  auto it_roi_prev_request = m_roi_prev_request.find(zoom_level);
  if (it_roi_prev_request == m_roi_prev_request.end())
  {
    // There was no previous request, so there can be no prediction which region of tiles might be needed in the next
    // processing step. Therefore set the current roi to be the prediction for the next request.
    m_roi_prediction[zoom_level] = roi_current;
  }
  else
  {
    // We have a previous roi that was requested, therefore we can extrapolate what the next request might look like
    // utilizing our current roi
    auto it_roi_prediction = m_roi_prediction.find(zoom_level);
    it_roi_prediction->second.x = roi_current.x + (roi_current.x - it_roi_prev_request->second.x);
    it_roi_prediction->second.y = roi_current.y + (roi_current.y - it_roi_prev_request->second.y);
    it_roi_prediction->second.width = roi_current.width + (roi_current.width - it_roi_prev_request->second.width);
    it_roi_prediction->second.height = roi_current.height + (roi_current.height - it_roi_prev_request->second.height);
  }

  it_roi_prev_request->second = roi_current;
}

void TileCache::createDirectories(const std::string &toplevel, const std::vector<std::string> &layer_names, const std::string &tile_tree)
{
  for (const auto &layer_name : layer_names)
  {
    io::createDir(toplevel + layer_name + tile_tree);
  }
}