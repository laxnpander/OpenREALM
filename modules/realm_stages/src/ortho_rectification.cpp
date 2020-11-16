/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
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

#include <realm_stages/ortho_rectification.h>

using namespace realm;
using namespace stages;

OrthoRectification::OrthoRectification(const StageSettings::Ptr &stage_set, double rate)
    : StageBase("ortho_rectification", (*stage_set)["path_output"].toString(), rate, (*stage_set)["queue_size"].toInt()),
      _do_publish_pointcloud((*stage_set)["publish_pointcloud"].toInt() > 0),
      _GSD((*stage_set)["GSD"].toDouble()),
      _settings_save({(*stage_set)["save_ortho_rgb"].toInt() > 0,
                  (*stage_set)["save_ortho_gtiff"].toInt() > 0,
                  (*stage_set)["save_elevation"].toInt() > 0,
                  (*stage_set)["save_elevation_angle"].toInt() > 0})
{
  std::cout << "Stage [" << _stage_name << "]: Created Stage with Settings: " << std::endl;
  stage_set->print();

  registerAsyncDataReadyFunctor([=]{ return !_buffer.empty(); });
}

void OrthoRectification::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateFpsStatisticsIncoming();

  if (!frame->getSurfaceModel())
  {
    LOG_F(INFO, "Input frame has no surface informations. Dropping...");
    return;
  }
  if (!frame->getSurfaceModel()->exists("elevation"))
  {
    LOG_F(INFO, "Input frame missing surface elevation layer. Dropping...");
    return;
  }
  std::unique_lock<std::mutex> lock(_mutex_buffer);
  _buffer.push_back(frame);
  // Ringbuffer implementation for buffer with no pose
  if (_buffer.size() > _queue_size)
    _buffer.pop_front();
}

bool OrthoRectification::process()
{
  bool has_processed = false;
  if (!_buffer.empty())
  {
    // Prepare timing
    long t;

    Frame::Ptr frame = getNewFrame();
    LOG_F(INFO, "Processing frame #%u...", frame->getFrameId());

    // Make deep copy of the surface model, so we can resize it later on
    CvGridMap::Ptr surface_model = frame->getSurfaceModel();

    double resize_quotient = surface_model->resolution() / _GSD;
    LOG_F(INFO, "Resize quotient rq = (elevation.resolution() / GSD) = %4.2f", resize_quotient);
    LOG_IF_F(INFO, resize_quotient < 0.9, "Loss of resolution! Consider downsizing depth map or increase GSD.");
    LOG_IF_F(INFO, resize_quotient > 1.1, "Large resizing of elevation map detected. Keep in mind that ortho resolution is now >> spatial resolution");

    // First change resolution of observed map to desired GSD
    t = getCurrentTimeMilliseconds();
    surface_model->changeResolution(_GSD);
    LOG_F(INFO, "Timing [Resizing]: %lu ms", getCurrentTimeMilliseconds()-t);

    // Rectification needs img data, surface map and camera pose -> All contained in frame
    // Output, therefore the new additional data is written into rectified map
    t = getCurrentTimeMilliseconds();
    CvGridMap::Ptr map_rectified = ortho::rectify(frame);
    LOG_F(INFO, "Timing [Rectify]: %lu ms", getCurrentTimeMilliseconds()-t);

    // The orthophoto is contained in the rectified output. However, there is plenty of other data that is better stored
    // inside the digital surface model
    t = getCurrentTimeMilliseconds();

    CvGridMap::Ptr orthophoto = std::make_shared<CvGridMap>(map_rectified->getSubmap({"color_rgb"}));
    frame->setOrthophoto(orthophoto);

    surface_model->add("elevation_angle", (*map_rectified)["elevation_angle"]);
    surface_model->add("elevated", (*map_rectified)["elevated"]);
    surface_model->add("num_observations", (*map_rectified)["num_observations"]);

    LOG_F(INFO, "Timing [Adding]: %lu ms", getCurrentTimeMilliseconds()-t);

    // Transport results
    t = getCurrentTimeMilliseconds();
    publish(frame);
    LOG_F(INFO, "Timing [Publish]: %lu ms", getCurrentTimeMilliseconds()-t);

    // Savings every iteration
    t = getCurrentTimeMilliseconds();
    saveIter(*surface_model, *orthophoto, frame->getGnssUtm().zone, frame->getGnssUtm().band, frame->getFrameId());
    LOG_F(INFO, "Timing [Saving]: %lu ms", getCurrentTimeMilliseconds()-t);

    has_processed = true;
  }
  return has_processed;
}

void OrthoRectification::reset()
{
  // TODO: Implement
}

void OrthoRectification::saveIter(const CvGridMap& surface_model, const CvGridMap &orthophoto, uint8_t zone, char band, uint32_t id)
{
  // check for NaN
  cv::Mat valid = (surface_model["elevation"] == surface_model["elevation"]);

  if (_settings_save.save_ortho_rgb)
    io::saveCvGridMapLayer(orthophoto, zone, band, "color_rgb", io::createFilename(_stage_path + "/ortho/ortho_", id, ".png"));
  if (_settings_save.save_elevation_angle)
    io::saveImageColorMap(surface_model["elevation_angle"], valid, _stage_path + "/angle", "angle", id, io::ColormapType::ELEVATION);
  if (_settings_save.save_ortho_gtiff)
    io::saveGeoTIFF(orthophoto.getSubmap({"color_rgb"}), zone, io::createFilename(_stage_path + "/gtiff/gtiff_", id, ".tif"));
  if (_settings_save.save_elevation)
    io::saveGeoTIFF(surface_model.getSubmap({"elevation"}), zone, io::createFilename(_stage_path + "/elevation/elevation_", id, ".tif"));
}

void OrthoRectification::publish(const Frame::Ptr &frame)
{
  // First update statistics about outgoing frame rate
  updateFpsStatisticsOutgoing();

  _transport_frame(frame, "output/frame");
  _transport_img((*frame->getOrthophoto())["color_rgb"], "output/rectified");

  if (_do_publish_pointcloud)
  {
    CvGridMap::Ptr surface_model = frame->getSurfaceModel();
    CvGridMap::Ptr orthophoto = frame->getOrthophoto();

    CvGridMap map(orthophoto->roi(), orthophoto->resolution());
    map.add(*surface_model, REALM_OVERWRITE_ALL, false);
    map.add(*orthophoto, REALM_OVERWRITE_ALL, false);

    // Check for NaN
    cv::Mat valid = ((*surface_model)["elevation"] == (*surface_model)["elevation"]);

    /*cv::Mat point_cloud;
    if (frame->getSurfaceModel()->exists("elevation_normal"))
      point_cloud = cvtToPointCloud(map, "elevation", "color_rgb", "elevation_normal", "valid");
    else
      point_cloud = cvtToPointCloud(map, "elevation", "color_rgb", "", "valid");
    _transport_pointcloud(point_cloud, "output/pointcloud");*/
  }
}


Frame::Ptr OrthoRectification::getNewFrame()
{
  std::unique_lock<std::mutex> lock(_mutex_buffer);
  Frame::Ptr frame = _buffer.front();
  _buffer.pop_front();
  return (std::move(frame));
}

void OrthoRectification::initStageCallback()
{
  // Stage directory first
  if (!io::dirExists(_stage_path))
    io::createDir(_stage_path);

  // Then sub directories
  if (!io::dirExists(_stage_path + "/elevation"))
    io::createDir(_stage_path + "/elevation");
  if (!io::dirExists(_stage_path + "/angle"))
    io::createDir(_stage_path + "/angle");
  if (!io::dirExists(_stage_path + "/gtiff"))
    io::createDir(_stage_path + "/gtiff");
  if (!io::dirExists(_stage_path + "/ortho"))
    io::createDir(_stage_path + "/ortho");
}

void OrthoRectification::printSettingsToLog()
{
  LOG_F(INFO, "### Stage process settings ###");
  LOG_F(INFO, "- GSD: %4.2f", _GSD);
  LOG_F(INFO, "- publish_pointcloud: %i", _do_publish_pointcloud);

  LOG_F(INFO, "### Stage save settings ###");
  LOG_F(INFO, "- save_ortho_rgb: %i", _settings_save.save_ortho_rgb);
  LOG_F(INFO, "- save_ortho_gtiff: %i", _settings_save.save_ortho_gtiff);
  LOG_F(INFO, "- save_elevation: %i", _settings_save.save_elevation);
  LOG_F(INFO, "- save_elevation_angle: %i", _settings_save.save_elevation_angle);
}