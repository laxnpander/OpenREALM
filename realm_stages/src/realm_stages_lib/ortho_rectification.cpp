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

#include <realm_common/loguru.hpp>

#include <realm_stages/ortho_rectification.h>

using namespace realm;
using namespace stages;

OrthoRectification::OrthoRectification(const StageSettings::Ptr &stage_set)
    : StageBase("ortho_rectification", stage_set->get<std::string>("path_output"), stage_set->get<int>("queue_size")),
      _GSD(stage_set->get<double>("GSD")),
      _settings_save({stage_set->get<int>("save_valid") > 0,
                      stage_set->get<int>("save_ortho_rgb") > 0,
                      stage_set->get<int>("save_ortho_gtiff") > 0,
                      stage_set->get<int>("save_elevation") > 0,
                      stage_set->get<int>("save_elevation_angle") > 0})
{
  std::cout << "Stage [" << _stage_name << "]: Created Stage with Settings: " << std::endl;
  stage_set->print();
}

void OrthoRectification::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateFpsStatisticsIncoming();

  if (!frame->hasObservedMap())
  {
    LOG_F(INFO, "Input frame has no surface informations. Dropping...");
    return;
  }
  if (!frame->getObservedMap()->exists("elevation"))
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
    Frame::Ptr frame = getNewFrame();
    LOG_F(INFO, "Processing frame #%llu...", frame->getFrameId());

    CvGridMap::Ptr observed_map = frame->getObservedMap();

    double resize_quotient = observed_map->resolution()/_GSD;
    LOG_F(INFO, "Resize quotient rq = (elevation.resolution() / GSD) = %4.2f", resize_quotient);
    LOG_IF_F(INFO, resize_quotient < 0.9, "Loss of resolution! Consider downsizing depth map or increase GSD.");
    LOG_IF_F(INFO, resize_quotient > 1.1, "Large resizing of elevation map detected. Keep in mind that ortho resolution is now >> spatial resolution");

    // Check ranges of input elevation, this is necessary to correct resizing interpolation errors
    double ele_min, ele_max;
    cv::Point2i min_loc, max_loc;
    cv::minMaxLoc((*observed_map)["elevation"], &ele_min, &ele_max, &min_loc, &max_loc, (*observed_map)["valid"]);

    // First change resolution of observed map to desired GSD
    observed_map->setLayerInterpolation("valid", CV_INTER_NN);
    observed_map->changeResolution(_GSD);

    // After resizing through bilinear interpolation there can occure bad elevation values at the border
    cv::Mat mask_low = ((*observed_map)["elevation"] < ele_min);
    cv::Mat mask_high = ((*observed_map)["elevation"] > ele_max);
    (*observed_map)["elevation"].setTo(consts::getNoValue<float>(), mask_low);
    (*observed_map)["elevation"].setTo(consts::getNoValue<float>(), mask_high);
    (*observed_map)["valid"].setTo(0, mask_low);
    (*observed_map)["valid"].setTo(0, mask_high);

    // Rectification needs img data, surface map and camera pose -> All contained in frame
    // Output, therefore the new additional data is written into rectified map
    CvGridMap map_rect;
    ortho::rectify(frame, map_rect);
    observed_map->add(map_rect);

    // Transport results
    publish(frame);

    // Savings every iteration
    saveIter(*observed_map, frame->getGnssUtm().zone, frame->getFrameId());

    has_processed = true;
  }
  return has_processed;
}

void OrthoRectification::reset()
{
  // TODO: Implement
}

void OrthoRectification::saveIter(const CvGridMap& map, uint8_t zone, uint32_t id)
{
  if (_settings_save.save_valid)
    io::saveImage(map["valid"], _stage_path + "/valid", "valid", id);
  if (_settings_save.save_ortho_rgb)
    io::saveImage(map["color_rgb"], _stage_path + "/ortho", "ortho", id);
  if (_settings_save.save_elevation_angle)
    io::saveImageColorMap(map["elevation_angle"], map["valid"], _stage_path + "/angle", "angle", id, io::ColormapType::ELEVATION);
  if (_settings_save.save_ortho_gtiff)
    io::saveGeoTIFF(map, "color_rgb", zone, _stage_path + "/gtiff", "gtiff", id);
  if (_settings_save.save_elevation)
    io::saveGeoTIFF(map, "elevation", zone, _stage_path + "/elevation", "elevation", id);
}

void OrthoRectification::publish(const Frame::Ptr &frame)
{
  // First update statistics about outgoing frame rate
  updateFpsStatisticsOutgoing();

  _transport_frame(frame, "output/frame");
  _transport_img((*frame->getObservedMap())["color_rgb"], "output/rectified");

  cv::Mat point_cloud;
  if (frame->getObservedMap()->exists("elevation_normal"))
    point_cloud = cvtToPointCloud(*frame->getObservedMap(), "elevation", "color_rgb", "elevation_normal", "valid");
  else
    point_cloud = cvtToPointCloud(*frame->getObservedMap(), "elevation", "color_rgb", "", "valid");
  _transport_pointcloud(point_cloud, "output/pointcloud");
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
  if (!io::dirExists(_stage_path + "/valid"))
    io::createDir(_stage_path + "/valid");
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

  LOG_F(INFO, "### Stage save settings ###");
  LOG_F(INFO, "- save_valid: %i", _settings_save.save_valid);
  LOG_F(INFO, "- save_ortho_rgb: %i", _settings_save.save_ortho_rgb);
  LOG_F(INFO, "- save_ortho_gtiff: %i", _settings_save.save_ortho_gtiff);
  LOG_F(INFO, "- save_elevation: %i", _settings_save.save_elevation);
  LOG_F(INFO, "- save_elevation_angle: %i", _settings_save.save_elevation_angle);
}