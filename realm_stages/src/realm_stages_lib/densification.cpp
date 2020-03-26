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

#include <realm_stages/densification.h>

using namespace realm;
using namespace stages;

Densification::Densification(const StageSettings::Ptr &stage_set,
                             const DensifierSettings::Ptr &densifier_set)
: StageBase("densification", stage_set->get<std::string>("path_output"), stage_set->get<int>("queue_size")),
  _use_sparse_depth(stage_set->get<int>("use_sparse_disparity") > 0),
  _use_filter_bilat(stage_set->get<int>("use_filter_bilat") > 0),
  _use_filter_guided(stage_set->get<int>("use_filter_guided") > 0),
  _compute_normals(stage_set->get<int>("compute_normals") > 0),
  _buffer_selector(_buffer_reco.end()),
  _rcvd_frames(0),
  _settings_save({stage_set->get<int>("save_bilat") > 0,
                  stage_set->get<int>("save_dense") > 0,
                  stage_set->get<int>("save_guided") > 0,
                  stage_set->get<int>("save_imgs") > 0,
                  stage_set->get<int>("save_sparse") > 0,
                  stage_set->get<int>("save_thumb") > 0,
                  stage_set->get<int>("save_normals") > 0})
{
  _densifier = densifier::DensifierFactory::create(densifier_set);
  _n_frames = _densifier->getNrofInputFrames();
  _use_dense_depth = (_n_frames > 0);

  bool no_densification = !(_use_sparse_depth || _use_dense_depth);

  LOG_IF_F(WARNING, no_densification, "Densification launched, but settings forbid.");
  LOG_IF_F(WARNING, no_densification, "Try set 'use_sparse_depth' or 'use_dense_depth' in settings. All frames are redirected.");
}

void Densification::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateFpsStatisticsIncoming();

  // Check if frame and settings are fulfilled to process/densify incoming frames
  // if not, redirect to next stage
  if (   !frame->isKeyframe()
      || !frame->hasAccuratePose()
      || frame->getSurfacePoints().empty()
      || !(_use_sparse_depth|| _use_dense_depth))
  {
    LOG_F(INFO, "Frame #%llu:", frame->getFrameId());
    LOG_F(INFO, (std::string("Keyframe? ") + (frame->isKeyframe() ? "Yes" : "No")).c_str());
    LOG_F(INFO, (std::string("Accurate Pose? ") + (frame->hasAccuratePose() ? "Yes" : "No")).c_str());
    LOG_F(INFO, "Surface? %i Points", frame->getSurfacePoints().rows);

    LOG_F(INFO, "Frame #%llu not suited for dense reconstruction. Passing through...", frame->getFrameId());
    _transport_frame(frame, "output/frame");
    return;
  }

  // Check what kind of processing should take place with incoming frame
  if (_use_dense_depth)
    pushToBufferReco(frame);

  // Case: Stereo reconstruction should be performed, but not sure if enough frames already
  if (_use_sparse_depth && _use_dense_depth)
    if (_rcvd_frames < _n_frames/2)
      pushToBufferNoReco(frame);

  // Case: Dummy densifier, only sparse cloud pseudo densification
  if (_use_sparse_depth && !_use_dense_depth)
    pushToBufferNoReco(frame);

  // Increment received valid frames
  _rcvd_frames++;
}

bool Densification::process()
{
  // NOTE: All depthmap maps are CV_32F except they are explicitly casted

  // First update current processing mode based on buffer elements
  ProcessingElement procc_element = getProcessingElement();

  if (procc_element.mode == ProcessingMode::IDLE)
    return false;

  // Processing step
  cv::Mat depthmap;
  if (procc_element.mode == ProcessingMode::NO_RECONSTRUCTION)
    if (!processNoReconstruction(procc_element.buffer, depthmap))
      return true;

  if (procc_element.mode == ProcessingMode::STEREO_RECONSTRUCTION)
    if (!processStereoReconstruction(procc_element.buffer, depthmap))
      return true;

  // Post processing steps
  cv::Mat depthmap_filtered = applyDepthMapPostProcessing(depthmap);

  cv::Mat normals;
  if (_compute_normals)
    normals = stereo::computeNormalsFromDepthMap(depthmap_filtered);

  // Output step
  LOG_F(INFO, "Scene depthmap force in range %4.2f ... %4.2f", _depth_min_curr, _depth_max_curr);

  cv::Mat mask;
  if (procc_element.mode == ProcessingMode::NO_RECONSTRUCTION)
    mask = computeDepthMapMask(depthmap_filtered, true);
  else
    mask = computeDepthMapMask(depthmap_filtered, false);

  if (_compute_normals)
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 4, cv::BORDER_CONSTANT, 0);

  LOG_F(INFO, "Reprojecting depthmap map into space...");
  cv::Mat img3d = stereo::reprojectDepthMap(_frame_curr->getResizedCamera(), depthmap_filtered);
  cv::Mat surface_pts = cvtToPointCloud(img3d, _frame_curr->getResizedImageUndistorted(), normals, mask);

  _frame_curr->setSurfacePoints(surface_pts);

  // Savings
  saveIter(_frame_curr, normals, mask);

  // Republish frame to next stage
  publish(_frame_curr, depthmap_filtered);
  return true;
}

bool Densification::processNoReconstruction(const FrameBuffer &buffer, cv::OutputArray depthmap)
{
  LOG_F(INFO, "Performing no reconstruction. Interpolation of sparse cloud...");

  _frame_curr = buffer.front();

  if (_frame_curr->getSurfacePoints().rows < 50 || !_use_sparse_depth)
  {
    LOG_F(INFO, "Interpolation of sparse cloud failed. Too few sparse points!");
    popFromBufferNoReco();
    return false;
  }

  _depth_min_curr = static_cast<float>(_frame_curr->getMedianSceneDepth())*0.25f;
  _depth_max_curr = static_cast<float>(_frame_curr->getMedianSceneDepth())*1.75f;

  LOG_F(INFO, "Processing frame #%llu...", _frame_curr->getFrameId());

  // Resize factor must be set, in case sparse only was chosen (densifier is dummy)
  if (!_frame_curr->isImageResizeSet())
    _frame_curr->setImageResizeFactor(_densifier->getResizeFactor());

  // Compute sparse depth map and save if neccessary
  cv::Mat depthmap_sparse;
  cv::Mat depthmap_sparse_densified;

  densifier::computeDepthMapFromSparseCloud(_frame_curr->getSurfacePoints(),
                                            _frame_curr->getResizedCamera(),
                                            depthmap_sparse_densified,
                                            depthmap_sparse);

  if (_settings_save.save_thumb)
    io::saveImageColorMap(depthmap_sparse, _depth_min_curr, _depth_max_curr, _stage_path + "/thumb", "thumb", _frame_curr->getFrameId(), io::ColormapType::DEPTH);
  if (_settings_save.save_sparse)
    io::saveDepthMap(depthmap_sparse_densified, _stage_path + "/sparse/sparse_%06i.tif", _frame_curr->getFrameId(), _depth_min_curr, _depth_max_curr);

  depthmap.assign(depthmap_sparse_densified);
  popFromBufferNoReco();
  return true;
}

bool Densification::processStereoReconstruction(const FrameBuffer &buffer, cv::OutputArray depthmap)
{
  LOG_F(INFO, "Performing stereo reconstruction...");

  // Reference frame is the one in the middle (if more than two)
  int ref_idx = (int)buffer.size()/2;
  _frame_curr = buffer[ref_idx];
  _depth_min_curr = static_cast<float>(_frame_curr->getMedianSceneDepth())*0.25f;
  _depth_max_curr = static_cast<float>(_frame_curr->getMedianSceneDepth())*1.75f;

  cv::Mat georef_newest = buffer.back()->getGeoreference();
  for (auto &f : buffer)
  {
    f->setGeoreference(georef_newest);
    f->updateGeographicPose();
  }

  LOG_F(INFO, "Reconstructing frame #%llu...", _frame_curr->getFrameId());
  cv::Mat depthmap_dense = _densifier->densify(buffer, (uint8_t)ref_idx);

  LOG_IF_F(INFO, !depthmap_dense.empty(), "Successfully reconstructed frame!");
  if (depthmap_dense.empty())
  {
    LOG_F(INFO, "Reconstruction failed!");
    pushToBufferNoReco(_frame_curr);
    popFromBufferReco(_frame_curr->getCameraId());
    return false;
  }

  // Saving raw
  if (_settings_save.save_dense)
    io::saveDepthMap(depthmap_dense, _stage_path + "/dense/dense_%06i.tif", _frame_curr->getFrameId(), _depth_min_curr, _depth_max_curr);

  depthmap.assign(depthmap_dense);
  popFromBufferReco(_frame_curr->getCameraId());
  return true;
}

cv::Mat Densification::applyDepthMapPostProcessing(const cv::Mat &depthmap)
{
  assert(!depthmap.empty());

  cv::Mat depthmap_filtered;
  if (_use_filter_bilat)
      cv::bilateralFilter(depthmap, depthmap_filtered, 5, 25, 25);

  if (_settings_save.save_bilat)
    io::saveImageColorMap(depthmap_filtered, _depth_min_curr, _depth_max_curr,  _stage_path + "/bilat", "bilat",
                          _frame_curr->getFrameId(), io::ColormapType::DEPTH);

  return depthmap_filtered;
}

cv::Mat Densification::computeDepthMapMask(const cv::Mat &depth_map, bool use_sparse_mask)
{
  cv::Mat mask1, mask2, mask3;

  if (use_sparse_mask)
    densifier::computeSparseMask(_frame_curr->getSurfacePoints(), _frame_curr->getResizedCamera(), mask1);
  else
    mask1 = cv::Mat::ones(depth_map.rows, depth_map.cols, CV_8UC1)*255;

  cv::inRange(depth_map, _depth_min_curr, _depth_max_curr, mask2);
  cv::bitwise_and(mask1, mask2, mask3);
  return mask3;
}

Densification::ProcessingElement Densification::getProcessingElement()
{
  FrameBuffer buffer_no_reco = getNewFrameBufferNoReco();
  FrameBuffer buffer_reco = getNewFramesBufferReco();

  Frame::Ptr frame_no_reco = nullptr;
  if (!buffer_no_reco.empty())
    frame_no_reco = buffer_no_reco.front();

  Frame::Ptr frame_reco = nullptr;
  if (_n_frames > 0 && buffer_reco.size() == _n_frames)
    frame_reco = buffer_reco[_n_frames/2];

  // Determine which frame has priority
  // - if none of them exists -> idle
  // - if one of them exists -> use the one
  // - if both of them exist -> use the older one
  Densification::ProcessingElement procc_element;
  if (frame_no_reco == nullptr && frame_reco == nullptr)
  {
    procc_element.mode = ProcessingMode::IDLE;
  }
  else if (frame_reco == nullptr)
  {
    procc_element.buffer = buffer_no_reco;
    procc_element.mode = ProcessingMode::NO_RECONSTRUCTION;
  }
  else if (frame_no_reco == nullptr)
  {
    procc_element.buffer = buffer_reco;
    procc_element.mode = ProcessingMode::STEREO_RECONSTRUCTION;
  }
  else
  {
    if (frame_no_reco->getTimestamp() < frame_reco->getTimestamp())
    {
      procc_element.buffer = buffer_no_reco;
      procc_element.mode = ProcessingMode::NO_RECONSTRUCTION;
    }
    else
    {
      procc_element.buffer = buffer_reco;
      procc_element.mode = ProcessingMode::STEREO_RECONSTRUCTION;
    }
  }

  return procc_element;
}

void Densification::publish(const Frame::Ptr &frame, const cv::Mat &depthmap)
{
  // First update statistics about outgoing frame rate
  updateFpsStatisticsOutgoing();

  _transport_frame(frame, "output/frame");
  _transport_pose(frame->getCamera().pose(), frame->getGnssUtm().zone, frame->getGnssUtm().band, "output/pose");
  _transport_img(frame->getResizedImageUndistorted(), "output/img_rectified");
  _transport_depth_map(depthmap, "output/depth");
  _transport_pointcloud(frame->getSurfacePoints(), "output/pointcloud");

  cv::Mat depthmap_display;
  cv::normalize(depthmap, depthmap_display, 0, 65535, CV_MINMAX, CV_16UC1, (depthmap > 0));
  _transport_img(depthmap_display, "output/depth_display");
}

void Densification::saveIter(const Frame::Ptr &frame, const cv::Mat &normals, const cv::Mat &mask)
{
  if (_settings_save.save_imgs)
    io::saveImage(frame->getResizedImageUndistorted(), _stage_path + "/imgs", "imgs", frame->getFrameId());
  if (_settings_save.save_normals && _compute_normals && !normals.empty())
    io::saveImageColorMap(normals, mask, _stage_path + "/normals", "normals", frame->getFrameId(), io::ColormapType::NORMALS);
}

void Densification::pushToBufferReco(const Frame::Ptr &frame)
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_reco);

  auto buffer = _buffer_reco.find(frame->getCameraId());
  if (buffer != _buffer_reco.end())
  {
    buffer->second->push_back(frame);

    // Limit incoming frames
    if (buffer->second->size() > _queue_size)
      buffer->second->pop_front();
  }
  else
  {
    std::deque<Frame::Ptr> buffer_new{frame};
    _buffer_reco.insert({frame->getCameraId(), std::make_shared<FrameBuffer>(buffer_new)});
  }
}

void Densification::pushToBufferNoReco(const Frame::Ptr &frame)
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_no_reco);
  _buffer_no_reco.push_back(frame);

  // Limit incoming frames
  if (_buffer_no_reco.size() > _queue_size)
    _buffer_no_reco.pop_front();
}

void Densification::popFromBufferNoReco()
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_no_reco);
  _buffer_no_reco.pop_front();
}

void Densification::popFromBufferReco(const std::string &buffer_name)
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_reco);
  _buffer_reco[buffer_name]->pop_front();
}

Densification::FrameBuffer Densification::getNewFrameBufferNoReco()
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_no_reco);
  return _buffer_no_reco;
}

Densification::FrameBuffer Densification::getNewFramesBufferReco()
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_reco);

  FrameBuffer frames;

  // Buffer reco has elements
  if (!_buffer_reco.empty())
  {
    auto selector_copy = _buffer_selector;
    do
    {
      if (_buffer_selector == _buffer_reco.end())
        _buffer_selector = _buffer_reco.begin();
      if ((*_buffer_selector->second).size() >= _n_frames)
        break;
      _buffer_selector++;
    }
    while(selector_copy != _buffer_selector);

    if (_buffer_selector != _buffer_reco.end())
      if ((*_buffer_selector->second).size() >= _n_frames)
      {
        for (uint8_t i = 0; i < _n_frames; ++i)
          frames.push_back((*_buffer_selector->second)[i]);
        _buffer_selector++;
      }
  }

  return frames;
}

void Densification::reset()
{
  // TODO: Reset in _densifier
  _rcvd_frames = 0;
  LOG_F(INFO, "Densification Stage: RESETED!");
}

void Densification::initStageCallback()
{
  // Stage directory first
  if (!io::dirExists(_stage_path))
    io::createDir(_stage_path);

  // Then sub directories
  if (!io::dirExists(_stage_path + "/sparse"))
    io::createDir(_stage_path + "/sparse");
  if (!io::dirExists(_stage_path + "/dense"))
    io::createDir(_stage_path + "/dense");
  if (!io::dirExists(_stage_path + "/bilat"))
    io::createDir(_stage_path + "/bilat");
  if (!io::dirExists(_stage_path + "/guided"))
    io::createDir(_stage_path + "/guided");
  if (!io::dirExists(_stage_path + "/normals"))
    io::createDir(_stage_path + "/normals");
  if (!io::dirExists(_stage_path + "/imgs"))
    io::createDir(_stage_path + "/imgs");
  if (!io::dirExists(_stage_path + "/thumb"))
    io::createDir(_stage_path + "/thumb");
}

void Densification::printSettingsToLog()
{
  LOG_F(INFO, "### Stage process settings ###");
  LOG_F(INFO, "- use_sparse_depth: %i", _use_sparse_depth);
  LOG_F(INFO, "- use_filter_bilat: %i", _use_filter_bilat);
  LOG_F(INFO, "- use_filter_guided: %i", _use_filter_guided);
  LOG_F(INFO, "- compute_normals: %i", _compute_normals);

  LOG_F(INFO, "### Stage save settings ###");
  LOG_F(INFO, "- save_bilat: %i", _settings_save.save_bilat);
  LOG_F(INFO, "- save_dense: %i", _settings_save.save_dense);
  LOG_F(INFO, "- save_guided: %i", _settings_save.save_guided);
  LOG_F(INFO, "- save_imgs: %i", _settings_save.save_imgs);
  LOG_F(INFO, "- save_normals: %i", _settings_save.save_normals);
  LOG_F(INFO, "- save_sparse: %i", _settings_save.save_sparse);
  LOG_F(INFO, "- save_thumb: %i", _settings_save.save_thumb);

  _densifier->printSettingsToLog();
}