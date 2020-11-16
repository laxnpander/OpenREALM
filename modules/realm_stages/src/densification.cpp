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
                             const DensifierSettings::Ptr &densifier_set,
                             double rate)
: StageBase("densification", (*stage_set)["path_output"].toString(), rate, (*stage_set)["queue_size"].toInt()),
  _use_filter_bilat((*stage_set)["use_filter_bilat"].toInt() > 0),
  _use_filter_guided((*stage_set)["use_filter_guided"].toInt() > 0),
  _depth_min_current(0.0),
  _depth_max_current(0.0),
  _compute_normals((*stage_set)["compute_normals"].toInt() > 0),
  _rcvd_frames(0),
  _settings_save({(*stage_set)["save_bilat"].toInt() > 0,
                  (*stage_set)["save_dense"].toInt() > 0,
                  (*stage_set)["save_guided"].toInt() > 0,
                  (*stage_set)["save_imgs"].toInt() > 0,
                  (*stage_set)["save_sparse"].toInt() > 0,
                  (*stage_set)["save_thumb"].toInt() > 0,
                  (*stage_set)["save_normals"].toInt() > 0})
{
  registerAsyncDataReadyFunctor([=]{ return !_buffer_reco.empty(); });

  _densifier = densifier::DensifierFactory::create(densifier_set);
  _n_frames = _densifier->getNrofInputFrames();

  // Creation of reference plane, currently only the one below is supported
  _plane_ref.pt = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
  _plane_ref.n = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
}

void Densification::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateFpsStatisticsIncoming();

  // Increment received valid frames
  _rcvd_frames++;

  // Check if frame and settings are fulfilled to process/densify incoming frames
  // if not, redirect to next stage
  if (   !frame->isKeyframe()
      || !frame->hasAccuratePose()
      || !frame->isDepthComputed())
  {
    LOG_F(INFO, "Frame #%u:", frame->getFrameId());
    LOG_F(INFO, "Keyframe? %s", frame->isKeyframe() ? "Yes" : "No");
    LOG_F(INFO, "Accurate Pose? %s", frame->hasAccuratePose() ? "Yes" : "No");
    LOG_F(INFO, "Surface? %i Points", frame->getSparseCloud().rows);

    LOG_F(INFO, "Frame #%u not suited for dense reconstruction. Passing through...", frame->getFrameId());
    _transport_frame(frame, "output/frame");
    return;
  }

  pushToBufferReco(frame);
}

bool Densification::process()
{
  // NOTE: All depthmap maps are CV_32F except they are explicitly casted

  // First check if buffer has enough frames already, else don't do anything
  if (_buffer_reco.size() < _n_frames)
  {
    return false;
  }

  // Densification step using stereo
  long t = getCurrentTimeMilliseconds();
  Frame::Ptr frame_processed;
  Depthmap::Ptr depthmap = processStereoReconstruction(_buffer_reco, frame_processed);
  popFromBufferReco();
  LOG_IF_F(INFO, _verbose, "Timing [Dense Reconstruction]: %lu ms", getCurrentTimeMilliseconds()-t);
  if (!depthmap)
    return true;

  // Compute normals if desired
  t = getCurrentTimeMilliseconds();
  cv::Mat normals;
  if (_compute_normals)
    normals = stereo::computeNormalsFromDepthMap(depthmap->data());
  LOG_IF_F(INFO, _verbose, "Timing [Computing Normals]: %lu ms", getCurrentTimeMilliseconds()-t);

  // Remove outliers
  double depth_min = frame_processed->getMedianSceneDepth()*0.25;
  double depth_max = frame_processed->getMedianSceneDepth()*1.75;
  depthmap = forceInRange(depthmap, depth_min, depth_max);
  LOG_F(INFO, "Scene depthmap forced in range %4.2f ... %4.2f", depth_min, depth_max);

  // Set data in the frame
  t = getCurrentTimeMilliseconds();
  frame_processed->setDepthmap(depthmap);
  LOG_IF_F(INFO, _verbose, "Timing [Setting]: %lu ms", getCurrentTimeMilliseconds()-t);

  // Creating dense cloud
  cv::Mat img3d = stereo::reprojectDepthMap(depthmap->getCamera(), depthmap->data());
  cv::Mat dense_cloud = img3d.reshape(1, img3d.rows*img3d.cols);

  // Denoising
  t = getCurrentTimeMilliseconds();
  _buffer_consistency.emplace_back(std::make_pair(frame_processed, dense_cloud));
  if (_buffer_consistency.size() >= 4)
  {
    frame_processed = consistencyFilter(&_buffer_consistency);
    _buffer_consistency.pop_front();
  }
  else
  {
    LOG_IF_F(INFO, _verbose, "Consistency filter is activated. Waiting for more frames for denoising...");
    return true;
  }
  LOG_IF_F(INFO, _verbose, "Timing [Denoising]: %lu ms", getCurrentTimeMilliseconds()-t);

  // Last check if frame still has valid depthmap
  if (!frame_processed->getDepthmap())
  {
    //_transport_frame(frame_processed, "output/frame");
    return true;
  }

  // Post processing
  depthmap->data() = applyDepthMapPostProcessing(depthmap->data());

  // Savings
  t = getCurrentTimeMilliseconds();
  saveIter(frame_processed, normals);
  LOG_IF_F(INFO, _verbose, "Timing [Saving]: %lu ms", getCurrentTimeMilliseconds()-t);

  // Republish frame to next stage
  t = getCurrentTimeMilliseconds();
  publish(frame_processed, depthmap->data());
  LOG_IF_F(INFO, _verbose, "Timing [Publish]: %lu ms", getCurrentTimeMilliseconds()-t);

  return true;
}

Frame::Ptr Densification::consistencyFilter(std::deque<std::pair<Frame::Ptr, cv::Mat>>* buffer_denoise)
{
  Frame::Ptr frame = (*buffer_denoise)[buffer_denoise->size()/2].first;

  Depthmap::Ptr depthmap_ii = frame->getDepthmap();

  cv::Mat depthmap_ii_data = depthmap_ii->data();
  int rows = depthmap_ii_data.rows;
  int cols = depthmap_ii_data.cols;

  cv::Mat votes = cv::Mat::zeros(rows, cols, CV_8UC1);

  float th_depth = 0.1;

  for (const auto &f : *buffer_denoise)
  {
    if (f.first == frame)
      continue;

    cv::Mat dense_cloud = f.second;
    cv::Mat depthmap_ij_data = stereo::computeDepthMapFromPointCloud(depthmap_ii->getCamera(), dense_cloud);

    for (int r = 0; r < rows; ++r)
      for (int c =0; c < cols; ++c)
      {
        float d_ii = depthmap_ii_data.at<float>(r, c);
        float d_ij = depthmap_ij_data.at<float>(r, c);

        if (d_ii <= 0 || d_ij <= 0)
          continue;

        if (fabsf(d_ij-d_ii)/d_ii < th_depth)
          votes.at<uchar>(r, c) = votes.at<uchar>(r, c) + 1;
      }
  }

  cv::Mat mask = (votes < 2);
  depthmap_ii_data.setTo(-1.0, mask);

  double perc_coverage = 100 - (static_cast<double>(cv::countNonZero(mask)) / (mask.rows*mask.cols) * 100.0);

  if (perc_coverage < 30.0)
  {
    LOG_IF_F(WARNING, _verbose, "Depthmap coverage too low (%3.1f%%). Assuming plane surface.");
    frame->setDepthmap(nullptr);

    for (auto it = buffer_denoise->begin(); it != buffer_denoise->end(); ) {
      if (it->first->getFrameId() == frame->getFrameId())
      {
        it = buffer_denoise->erase(it);
        break;
      }
      else
        ++it;
    }
  }
  else
  {
    LOG_IF_F(INFO, _verbose, "Depthmap coverage left after denoising: %3.1f%%", perc_coverage);
  }

  return frame;
}

Depthmap::Ptr Densification::processStereoReconstruction(const std::deque<Frame::Ptr> &buffer, Frame::Ptr &frame_processed)
{
  LOG_F(INFO, "Performing stereo reconstruction...");

  // Reference frame is the one in the middle (if more than two)
  int ref_idx = (int)buffer.size()/2;
  frame_processed = buffer[ref_idx];

  // Compute baseline information for all frames
  std::vector<double> baselines;
  baselines.reserve(buffer.size());

  std::string stringbuffer;
  for (auto &f : buffer)
  {
    if (f == frame_processed)
      continue;

    baselines.push_back(stereo::computeBaselineFromPose(frame_processed->getPose(), f->getPose()));
    stringbuffer += std::to_string(baselines.back()) + "m ";
  }
  LOG_F(INFO, "Baselines to reference frame: %s", stringbuffer.c_str());

  LOG_F(INFO, "Reconstructing frame #%u...", frame_processed->getFrameId());
  Depthmap::Ptr depthmap = _densifier->densify(buffer, (uint8_t)ref_idx);

  LOG_IF_F(INFO, depthmap != nullptr, "Successfully reconstructed frame!");
  LOG_IF_F(WARNING, depthmap == nullptr, "Reconstruction failed!");

  return depthmap;
}

Depthmap::Ptr Densification::forceInRange(const Depthmap::Ptr &depthmap, double min_depth, double max_depth)
{
  cv::Mat mask;
  cv::Mat data = depthmap->data();

  cv::inRange(data, min_depth, max_depth, mask);

  cv::bitwise_not(mask, mask);
  data.setTo(-1.0f, mask);

  return depthmap;
}

cv::Mat Densification::applyDepthMapPostProcessing(const cv::Mat &depthmap)
{
  cv::Mat depthmap_filtered;
  if (_use_filter_bilat)
      cv::bilateralFilter(depthmap, depthmap_filtered, 5, 25, 25);

  /*if (_settings_save.save_bilat)
    io::saveImageColorMap(depthmap_filtered, _depth_min_current, _depth_max_current, _stage_path + "/bilat", "bilat",
                          _frame_current->getFrameId(), io::ColormapType::DEPTH);*/

  return depthmap_filtered;
}

cv::Mat Densification::computeDepthMapMask(const cv::Mat &depth_map, bool use_sparse_mask)
{
  cv::Mat mask1, mask2, mask3;

  /*if (use_sparse_mask)
    densifier::computeSparseMask(_frame_current->getSparseCloud(), _frame_current->getResizedCamera(), mask1);
  else
    mask1 = cv::Mat::ones(depth_map.rows, depth_map.cols, CV_8UC1)*255;

  cv::inRange(depth_map, _depth_min_current, _depth_max_current, mask2);
  cv::bitwise_and(mask1, mask2, mask3);*/
  return mask3;
}

void Densification::publish(const Frame::Ptr &frame, const cv::Mat &depthmap)
{
  // First update statistics about outgoing frame rate
  updateFpsStatisticsOutgoing();

  _transport_frame(frame, "output/frame");
  _transport_pose(frame->getPose(), frame->getGnssUtm().zone, frame->getGnssUtm().band, "output/pose");
  _transport_img(frame->getResizedImageUndistorted(), "output/img_rectified");
  _transport_depth_map(depthmap, "output/depth");
  _transport_pointcloud(frame->getSparseCloud(), "output/pointcloud");

  cv::Mat depthmap_display;
  cv::normalize(depthmap, depthmap_display, 0, 65535, CV_MINMAX, CV_16UC1, (depthmap > 0));
  _transport_img(depthmap_display, "output/depth_display");
}

void Densification::saveIter(const Frame::Ptr &frame, const cv::Mat &normals)
{
  cv::Mat depthmap_data = frame->getDepthmap()->data();

  if (_settings_save.save_imgs)
    io::saveImage(frame->getResizedImageUndistorted(), io::createFilename(_stage_path + "/imgs/imgs_", frame->getFrameId(), ".png"));
  if (_settings_save.save_normals && _compute_normals && !normals.empty())
    io::saveImageColorMap(normals, (depthmap_data > 0), _stage_path + "/normals", "normals", frame->getFrameId(), io::ColormapType::NORMALS);
  if (_settings_save.save_sparse)
  {
    cv::Mat depthmap_sparse = stereo::computeDepthMapFromPointCloud(frame->getResizedCamera(), frame->getSparseCloud().colRange(0, 3));
    io::saveDepthMap(depthmap_sparse,_stage_path + "/sparse/sparse_%06i.tif", frame->getFrameId());
  }
  if (_settings_save.save_dense)
    io::saveDepthMap(depthmap_data, _stage_path + "/dense/dense_%06i.tif", frame->getFrameId());
}

void Densification::pushToBufferReco(const Frame::Ptr &frame)
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_reco);

  _buffer_reco.push_back(frame);

  if (_buffer_reco.size() > _queue_size)
  {
    _buffer_reco.pop_front();
  }
}

void Densification::popFromBufferReco()
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_reco);
  _buffer_reco.pop_front();
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