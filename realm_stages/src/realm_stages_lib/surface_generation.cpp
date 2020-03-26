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

#include <realm_stages/surface_generation.h>

using namespace realm;
using namespace stages;

SurfaceGeneration::SurfaceGeneration(const StageSettings::Ptr &stage_set)
: StageBase("surface_generation", stage_set->get<std::string>("path_output"), stage_set->get<int>("queue_size")),
  _try_use_elevation(stage_set->get<int>("try_use_elevation") > 0),
  _knn_radius_factor(stage_set->get<double>("knn_radius_factor")),
  _mode_surface_normals(static_cast<DigitalSurfaceModel::SurfaceNormalMode>(stage_set->get<int>("mode_surface_normals"))),
  _plane_reference(Plane{(cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0), (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0)}),
  _settings_save({stage_set->get<int>("save_valid") > 0,
                  stage_set->get<int>("save_elevation") > 0,
                  stage_set->get<int>("save_normals") > 0})
{
}

void SurfaceGeneration::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateFpsStatisticsIncoming();

  std::unique_lock<std::mutex> lock(_mutex_buffer);
  _buffer.push_back(frame);
  // Ringbuffer implementation
  if (_buffer.size() > _queue_size)
    _buffer.pop_front();
}

bool SurfaceGeneration::process()
{
  bool has_processed = false;
  if (!_buffer.empty())
  {
    Frame::Ptr frame = getNewFrame();
    LOG_F(INFO, "Processing frame #%llu...", frame->getFrameId());

    // Identify surface assumption for input frame and compute DSM
    DigitalSurfaceModel::Ptr dsm;
    SurfaceAssumption assumption = computeSurfaceAssumption(frame);
    switch(assumption)
    {
      case SurfaceAssumption::PLANAR:
        LOG_F(INFO, "Surface assumption: PLANAR.");
        dsm = createPlanarSurface(frame);
        break;
      case SurfaceAssumption::ELEVATION:
        LOG_F(INFO, "Surface assumption: ELEVATION.");
        dsm = createElevationSurface(frame);
        break;
    }

    CvGridMap::Ptr surface = dsm->getSurfaceGrid();

    // Observed map should be empty at this point, but check before set
    if (!frame->hasObservedMap())
      frame->setObservedMap(surface);
    else
      frame->getObservedMap()->add(*surface, REALM_OVERWRITE_ALL, true);
    frame->setSurfaceAssumption(assumption);

    LOG_F(INFO, "Publishing frame for next stage...");

    // Publishes every iteration
    publish(frame);

    // Savings every iteration
    saveIter(*surface, frame->getFrameId());

    has_processed = true;
  }
  return has_processed;
}

bool SurfaceGeneration::changeParam(const std::string& name, const std::string &val)
{
  std::unique_lock<std::mutex> lock(_mutex_params);
  if (name == "try_use_elevation")
  {
    _try_use_elevation = (val == "true" || val == "1" ? true : false);
    return true;
  }
  return false;
}

void SurfaceGeneration::reset()
{

}

void SurfaceGeneration::publish(const Frame::Ptr &frame)
{
  // First update statistics about outgoing frame rate
  updateFpsStatisticsOutgoing();
  _transport_frame(frame, "output/frame");
}

void SurfaceGeneration::saveIter(const CvGridMap &surface, uint32_t id)
{
  if (_settings_save.save_valid)
    io::saveImage(surface["valid"], _stage_path + "/valid", "valid", id);
  if (_settings_save.save_elevation)
    io::saveImageColorMap(surface["elevation"], surface["valid"], _stage_path + "/elevation", "elevation", id, io::ColormapType::ELEVATION);
  if (_settings_save.save_normals && surface.exists("elevation_normal"))
    io::saveImageColorMap(surface["elevation_normal"], surface["valid"], _stage_path + "/normals", "normal", id, io::ColormapType::NORMALS);
}

void SurfaceGeneration::initStageCallback()
{
  // Stage directory first
  if (!io::dirExists(_stage_path))
    io::createDir(_stage_path);

  // Then sub directories
  if (!io::dirExists(_stage_path + "/elevation"))
    io::createDir(_stage_path + "/elevation");
  if (!io::dirExists(_stage_path + "/normals"))
    io::createDir(_stage_path + "/normals");
  if (!io::dirExists(_stage_path + "/valid"))
    io::createDir(_stage_path + "/valid");
}

void SurfaceGeneration::printSettingsToLog()
{
  LOG_F(INFO, "### Stage process settings ###");
  LOG_F(INFO, "- try_use_elevation: %i", _try_use_elevation);
  LOG_F(INFO, "- knn_radius_factor: %4.2f", _knn_radius_factor);
  LOG_F(INFO, "- mode_surface_normals: %i", static_cast<int>(_mode_surface_normals));

  LOG_F(INFO, "### Stage save settings ###");
  LOG_F(INFO, "- save_valid: %i", _settings_save.save_valid);
  LOG_F(INFO, "- save_elevation: %i", _settings_save.save_elevation);
  LOG_F(INFO, "- save_normals: %i", _settings_save.save_normals);

}

Frame::Ptr SurfaceGeneration::getNewFrame()
{
  std::unique_lock<std::mutex> lock(_mutex_buffer);
  Frame::Ptr frame = _buffer.front();
  _buffer.pop_front();
  return (std::move(frame));
}

SurfaceAssumption SurfaceGeneration::computeSurfaceAssumption(const Frame::Ptr &frame)
{
  std::unique_lock<std::mutex> lock(_mutex_params);
  if (_try_use_elevation && frame->isKeyframe() && frame->hasAccuratePose())
  {
    LOG_F(INFO, "Frame is accurate and keyframe. Checking surface points...");
    cv::Mat surface_pts = frame->getSurfacePoints();
    LOG_F(INFO, "Detected %i surface points.", surface_pts.rows);
    if (surface_pts.rows > 50)
      return SurfaceAssumption::ELEVATION;
  }
  return SurfaceAssumption::PLANAR;
}

DigitalSurfaceModel::Ptr SurfaceGeneration::createPlanarSurface(const Frame::Ptr &frame)
{
  // Create planar surface in world frame
  cv::Rect2d roi = frame->getCamera().projectImageBoundsToPlaneRoi(_plane_reference.pt, _plane_reference.n);
  auto dsm = std::make_shared<DigitalSurfaceModel>(roi);
  return dsm;
}

DigitalSurfaceModel::Ptr SurfaceGeneration::createElevationSurface(const Frame::Ptr &frame)
{
  // Create elevated 2.5D surface in world frame
  cv::Rect2d roi = frame->getCamera().projectImageBoundsToPlaneRoi(_plane_reference.pt, _plane_reference.n);
  auto dsm = std::make_shared<DigitalSurfaceModel>(roi, frame->getSurfacePoints(), _mode_surface_normals, _knn_radius_factor);
  return dsm;
}