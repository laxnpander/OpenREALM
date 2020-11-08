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

#define LOGURU_WITH_STREAMS 1

#include <realm_stages/pose_estimation.h>

using namespace realm;
using namespace stages;

PoseEstimation::PoseEstimation(const StageSettings::Ptr &stage_set,
                               const VisualSlamSettings::Ptr &vslam_set,
                               const CameraSettings::Ptr &cam_set,
                               double rate)
    : StageBase("pose_estimation", (*stage_set)["path_output"].toString(), rate, (*stage_set)["queue_size"].toInt()),
      _is_georef_initialized(false),
      _use_vslam((*stage_set)["use_vslam"].toInt() > 0),
      _set_all_frames_keyframes((*stage_set)["set_all_frames_keyframes"].toInt() > 0),
      _strategy_fallback(PoseEstimation::FallbackStrategy((*stage_set)["fallback_strategy"].toInt())),
      _use_fallback(false),
      _use_initial_guess((*stage_set)["use_initial_guess"].toInt() > 0),
      _do_update_georef((*stage_set)["update_georef"].toInt() > 0),
      _do_suppress_outdated_pose_pub((*stage_set)["suppress_outdated_pose_pub"].toInt() > 0),
      _th_error_georef((*stage_set)["th_error_georef"].toDouble()),
      _overlap_max((*stage_set)["overlap_max"].toDouble()),
      _overlap_max_fallback((*stage_set)["overlap_max_fallback"].toDouble()),
      _settings_save({(*stage_set)["save_trajectory_gnss"].toInt() > 0,
                      (*stage_set)["save_trajectory_visual"].toInt() > 0,
                      (*stage_set)["save_frames"].toInt() > 0,
                      (*stage_set)["save_keyframes"].toInt() > 0,
                      (*stage_set)["save_keyframes_full"].toInt() > 0})
{
  if (_use_vslam)
  {
    _vslam = VisualSlamFactory::create(vslam_set, cam_set);

    // Set reset callback from vSLAM to this node
    // therefore if SLAM resets itself, node is being informed
    std::function<void(void)> reset_func = std::bind(&PoseEstimation::reset, this);
    _vslam->registerResetCallback(reset_func);

    // Set pose update callback
    namespace ph = std::placeholders;
    VisualSlamIF::PoseUpdateFuncCb update_func = std::bind(&PoseEstimation::updateKeyframeCb, this, ph::_1, ph::_2, ph::_3);
    _vslam->registerUpdateTransport(update_func);

    // Create geo reference initializer
    _georeferencer = std::make_shared<GeometricReferencer>(_th_error_georef);
  }

  evaluateFallbackStrategy(_strategy_fallback);

  // Create Pose Estimation publisher
  _stage_publisher.reset(new PoseEstimationIO(this, rate, true));
  _stage_publisher->start();

  // Creation of reference plane, currently only the one below is supported
  _plane_ref.pt = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
  _plane_ref.n = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);

  // Previous roi initialization
  _roi_prev = cv::Rect2d(0.0, 0.0, 0.0, 0.0);
}

PoseEstimation::~PoseEstimation()
{
  _stage_publisher->requestFinish();
  _stage_publisher->join();
}

void PoseEstimation::evaluateFallbackStrategy(PoseEstimation::FallbackStrategy strategy)
{
  LOG_F(INFO, "Evaluating fallback strategy...");

  switch (strategy)
  {
    case FallbackStrategy::ALWAYS:
      LOG_F(INFO, "Selected: ALWAYS - Images will be projected whenever tracking is lost.");
      _use_fallback = true;
      break;
    case FallbackStrategy::NEVER:
      _use_fallback = false;
      LOG_F(INFO, "Selected: NEVER - Image projection will not be used.");
      break;
    default:
      throw(std::invalid_argument("Error: Unknown fallback strategy!"));
  }
}

void PoseEstimation::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateFpsStatisticsIncoming();

  // The user can provide a-priori georeferencing. Check if this is the case
  if (!_is_georef_initialized && frame->isGeoreferenced())
  {
    LOG_F(INFO, "Detected a-priori georeference for frame %u. Assuming all frames are georeferenced.", frame->getFrameId());
    _georeferencer = std::make_shared<DummyReferencer>(frame->getGeoreference());
  }

  // Push to buffer for visual tracking
  if (_use_vslam)
    pushToBufferNoPose(frame);
  else
    pushToBufferPublish(frame);

  // Ringbuffer implementation for buffer with no pose
  if (_buffer_no_pose.size() > 5)
  {
    std::unique_lock<std::mutex> lock(_mutex_buffer_no_pose);
    _buffer_no_pose.pop_front();
  }

  _transport_pose(frame->getDefaultPose(), frame->getGnssUtm().zone, frame->getGnssUtm().band, "output/pose/gnss");
}

bool PoseEstimation::process()
{
  // Trigger; true if there happened any processing in this cycle.
  bool has_processed = false;

  // Prepare timing
  long t;

  // Grab georeference flag once at the beginning, to avoid multithreading problems
  if (_use_vslam)
    _is_georef_initialized = _georeferencer->isInitialized();

  // Process new frames without a visual pose currently
  if (!_buffer_no_pose.empty())
  {
    // Grab frame from buffer with no poses
    Frame::Ptr frame = getNewFrameTracking();

    LOG_F(INFO, "Processing frame #%i with timestamp %lu!", frame->getFrameId(), frame->getTimestamp());

    // Track current frame -> compute visual accurate pose
    t = getCurrentTimeMilliseconds();
    track(frame);
    LOG_F(INFO, "Timing [Tracking]: %lu ms", getCurrentTimeMilliseconds()-t);

    // Identify buffer for push
    if (frame->hasAccuratePose())
    {
      // Branch accurate pose and georef initialized
      if (_is_georef_initialized)
      {
        if (_do_update_georef && !_georeferencer->isBuisy())
        {
          std::thread t(std::bind(&GeospatialReferencerIF::update, _georeferencer, frame));
          t.detach();
        }
        pushToBufferAll(frame);
      }
    }
    if (frame->isKeyframe() && !_is_georef_initialized)
    {
      pushToBufferAll(frame);
      pushToBufferInit(frame);
    }

    // Data was processed during this loop
    has_processed = true;
  }

  // Handles georeference initialization and georeferencing of frame poses
  // but only starts, if a new frame was processed during this loop
  if (_use_vslam && has_processed)
  {
    if (!_is_georef_initialized && !_buffer_pose_init.empty() && !_georeferencer->isBuisy())
    {
      // Branch: Georef is not calculated yet
      LOG_F(INFO, "Size of init buffer: %lu", _buffer_pose_init.size());
      std::thread t(std::bind(&GeospatialReferencerIF::init, _georeferencer, _buffer_pose_init));
      t.detach();
      has_processed = true;
    }
    else if (_is_georef_initialized && !_buffer_pose_all.empty())
    {
      // Branch: Georef was successfully initialized and data waits to be georeferenced
      // Process all measurements in seperate thread
      if (!_buffer_pose_init.empty())
        _buffer_pose_init.clear();

      std::thread t(std::bind(&PoseEstimation::applyGeoreferenceToBuffer, this));
      t.detach();
      has_processed = true;
    }
  }
  return has_processed;
}

void PoseEstimation::track(Frame::Ptr &frame)
{
  LOG_F(INFO, "Tracking frame #%i in visual SLAM...!", frame->getFrameId());

  // Check if initial guess should be computed
  cv::Mat T_c2w_initial;
  if (_use_initial_guess && _georeferencer->isInitialized())
  {
    LOG_F(INFO, "Computing initial guess of current pose...");
    T_c2w_initial = computeInitialPoseGuess(frame);
  }

  // Compute visual pose
  _mutex_vslam.lock();
  VisualSlamIF::State state = _vslam->track(frame, T_c2w_initial);
  _mutex_vslam.unlock();

  if (_set_all_frames_keyframes && (state == VisualSlamIF::State::FRAME_INSERT))
  {
    state = VisualSlamIF::State::KEYFRAME_INSERT;
  }

  // Identify SLAM state
  switch (state)
  {
    case VisualSlamIF::State::LOST:
      if (estimatePercOverlap(frame) < _overlap_max_fallback)
        pushToBufferPublish(frame);
      LOG_F(WARNING, "No tracking.");
      break;
    case VisualSlamIF::State::INITIALIZED:
      LOG_F(INFO, "Visual SLAM initialized.");
      break;
    case VisualSlamIF::State::FRAME_INSERT:
      LOG_F(INFO, "Frame insertion.");
      break;
    case VisualSlamIF::State::KEYFRAME_INSERT:
      frame->setKeyframe(true);
      LOG_F(INFO, "Key frame insertion.");
      break;
  }
  // Save tracked img with features in member
  std::unique_lock<std::mutex> lock(_mutex_img_debug);
  _vslam->drawTrackedImage(_img_debug);
}

void PoseEstimation::reset()
{
  std::unique_lock<std::mutex> lock(_mutex_reset_requested);
  std::unique_lock<std::mutex> lock1(_mutex_buffer_no_pose);
  std::unique_lock<std::mutex> lock2(_mutex_buffer_pose_init);
  std::unique_lock<std::mutex> lock3(_mutex_buffer_pose_all);
  if (_reset_requested)  // User reset
  {
    LOG_F(INFO, "Reset has been requested!");
    // reset visual slam
    _mutex_vslam.lock();
    _vslam->reset();
    _mutex_vslam.unlock();
  }
  else  // visual slam reset
  {
    LOG_F(INFO, "Visual SLAM has triggered reset!");
    // In case of reset by vslam, publish all images in buffer with
    // default pose if overlap is less than defined
    for (auto const &frame : _buffer_pose_all)
    {
      frame->setPoseAccurate(false);
      frame->setKeyframe(false);
      pushToBufferPublish(frame);
    }
  }
  // Clear all buffers except the publisher buffer
  _buffer_pose_all.clear();
  _buffer_no_pose.clear();
  _buffer_pose_init.clear();

  // Reset georeferencing
  if (_use_vslam)
    _georeferencer.reset(new GeometricReferencer(_th_error_georef));
  _stage_publisher->requestReset();
  _is_georef_initialized = false;
  _reset_requested = false;
}

void PoseEstimation::queueImuData(const VisualSlamIF::ImuData &imu) const
{
  _vslam->queueImuData(imu);
}

bool PoseEstimation::changeParam(const std::string& name, const std::string &val)
{
  std::unique_lock<std::mutex> lock;
  if (name == "use_vslam")
  {
    _use_vslam = (val == "true" || val == "1");
    return true;
  }
  return false;
}

void PoseEstimation::initStageCallback()
{
  // Stage directory first
  if (!io::dirExists(_stage_path))
    io::createDir(_stage_path);

  // Then sub directories
  if (!io::dirExists(_stage_path + "/trajectory"))
    io::createDir(_stage_path + "/trajectory");
  if (!io::dirExists(_stage_path + "/keyframes"))
    io::createDir(_stage_path + "/keyframes");
  if (!io::dirExists(_stage_path + "/keyframes_full"))
    io::createDir(_stage_path + "/keyframes_full");
  if (!io::dirExists(_stage_path + "/frames"))
    io::createDir(_stage_path + "/frames");

  _stage_publisher->setOutputPath(_stage_path);
}

void PoseEstimation::printSettingsToLog()
{
  LOG_F(INFO, "### Stage process settings ###");
  LOG_F(INFO, "- use_vslam: %i", _use_vslam);
  LOG_F(INFO, "- use_fallback: %i", _use_fallback);
  LOG_F(INFO, "- do_update_georef: %i", _do_update_georef);
  LOG_F(INFO, "- do_suppress_outdated_pose_pub: %i", _do_suppress_outdated_pose_pub);
  LOG_F(INFO, "- th_error_georef: %4.2f", _th_error_georef);
  LOG_F(INFO, "- overlap_max: %4.2f", _overlap_max);
  LOG_F(INFO, "- overlap_max_fallback: %4.2f", _overlap_max_fallback);

  LOG_F(INFO, "### Stage save settings ###");
  LOG_F(INFO, "- save_trajectory_gnss: %i", _settings_save.save_trajectory_gnss);
  LOG_F(INFO, "- save_trajectory_visual: %i", _settings_save.save_trajectory_visual);
  LOG_F(INFO, "- save_frames: %i", _settings_save.save_frames);
  LOG_F(INFO, "- save_keyframes: %i", _settings_save.save_keyframes);
  LOG_F(INFO, "- save_keyframes_full: %i", _settings_save.save_keyframes_full);

  if (_use_vslam)
    _vslam->printSettingsToLog();
}

void PoseEstimation::pushToBufferNoPose(const Frame::Ptr &frame)
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_no_pose);
  _buffer_no_pose.push_back(frame);
}

void PoseEstimation::pushToBufferInit(const Frame::Ptr &frame)
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_pose_init);
  _buffer_pose_init.push_back(frame);
}

void PoseEstimation::pushToBufferAll(const Frame::Ptr &frame)
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_pose_all);
  _buffer_pose_all.push_back(frame);
}

void PoseEstimation::pushToBufferPublish(const Frame::Ptr &frame)
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_do_publish);
  _buffer_do_publish.push_back(frame);
}

void PoseEstimation::updatePreviousRoi(const Frame::Ptr &frame)
{
  _roi_prev = frame->getCamera()->projectImageBoundsToPlaneRoi(_plane_ref.pt, _plane_ref.n);
}

void PoseEstimation::updateKeyframeCb(int id, const cv::Mat &pose, const cv::Mat &points)
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_pose_all);

  // Find frame in "all" buffer for updating
  for (auto &frame : _buffer_pose_all)
    if (frame->getFrameId() == (uint32_t)id)
    {
      if (!pose.empty())
        frame->setVisualPose(pose);
      if (!points.empty())
        frame->setSparseCloud(points, true);
      frame->setKeyframe(true);
    }

  for (auto &frame : _buffer_do_publish)
    if (frame->getFrameId() == (uint32_t)id)
    {
      if (!pose.empty())
        frame->setVisualPose(pose);
      //if (!points.empty())
      //  frame->setSurfacePoints(points);
      //frame->setKeyframe(true);
    }
}

double PoseEstimation::estimatePercOverlap(const Frame::Ptr &frame)
{
  cv::Rect2d roi_curr = frame->getCamera()->projectImageBoundsToPlaneRoi(_plane_ref.pt, _plane_ref.n);
  return ((roi_curr & _roi_prev).area() / roi_curr.area())*100;
}

Frame::Ptr PoseEstimation::getNewFrameTracking()
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_no_pose);
  Frame::Ptr frame = _buffer_no_pose.front();
  _buffer_no_pose.pop_front();
  return std::move(frame);
}

Frame::Ptr PoseEstimation::getNewFramePublish()
{
  std::unique_lock<std::mutex> lock(_mutex_buffer_do_publish);
  Frame::Ptr frame = _buffer_do_publish.front();
  _buffer_do_publish.pop_front();
  return std::move(frame);
}

void PoseEstimation::applyGeoreferenceToBuffer()
{
  // Grab estimated georeference
  _mutex_t_w2g.lock();
  _T_w2g = _georeferencer->getTransformation();
  _mutex_t_w2g.unlock();

  // Apply estimated georeference to all measurements in the buffer
  while(!_buffer_pose_all.empty())
  {
    _mutex_buffer_pose_all.lock();
    Frame::Ptr frame = _buffer_pose_all.front();
    _buffer_pose_all.pop_front();
    _mutex_buffer_pose_all.unlock();

    // But check first, if frame has actually a visually estimated pose information
    // In case of default GNSS pose generated from lat/lon/alt/heading, pose is already in world frame
    if (frame->hasAccuratePose())
    {
      frame->initGeoreference(_T_w2g);
    }

    pushToBufferPublish(frame);
  }
}

cv::Mat PoseEstimation::computeInitialPoseGuess(const Frame::Ptr &frame)
{
  cv::Mat default_pose = frame->getDefaultPose();
  cv::Mat T_w2g = _georeferencer->getTransformation();
  T_w2g.pop_back();

  // Compute scale of georeference
  double sx = cv::norm(T_w2g.col(0));
  double sy = cv::norm(T_w2g.col(1));
  double sz = cv::norm(T_w2g.col(2));

  // Remove scale from georeference
  T_w2g.col(0) /= sx;
  T_w2g.col(1) /= sy;
  T_w2g.col(2) /= sz;

  // Invert transformation from world to global frame
  cv::Mat T_g2w = cv::Mat::eye(4, 4, T_w2g.type());
  cv::Mat R_t = (T_w2g.rowRange(0, 3).colRange(0, 3)).t();
  cv::Mat t = -R_t*T_w2g.rowRange(0, 3).col(3);
  R_t.copyTo(T_g2w.rowRange(0, 3).colRange(0, 3));
  t.copyTo(T_g2w.rowRange(0, 3).col(3));

  // Add row for homogenous coordinates
  cv::Mat hom = (cv::Mat_<double>(1, 4) << 0.0, 0.0, 0.0, 1.0);
  default_pose.push_back(hom);

  // Finally transform default pose from geographic into the world coordinate frame
  cv::Mat default_pose_in_world = T_g2w * default_pose;

  // Apply scale change
  default_pose_in_world.at<double>(0, 3) = default_pose_in_world.at<double>(0, 3) / sx;
  default_pose_in_world.at<double>(1, 3) = default_pose_in_world.at<double>(1, 3) / sy;
  default_pose_in_world.at<double>(2, 3) = default_pose_in_world.at<double>(2, 3) / sz;

  return default_pose_in_world.rowRange(0, 3);
}

void PoseEstimation::printGeoReferenceInfo(const Frame::Ptr &frame)
{
  UTMPose utm = frame->getGnssUtm();
  cv::Mat t = frame->getCamera()->t();

  LOG_F(INFO, "Georeferenced pose:");
  LOG_F(INFO, "GNSS: [%10.2f, %10.2f, %4.2f]", utm.easting, utm.northing, utm.altitude);
  LOG_F(INFO, "Visual: [%10.2f, %10.2f, %4.2f]", t.at<double>(0), t.at<double>(1), t.at<double>(2));
  LOG_F(INFO, "Diff: [%10.2f, %10.2f, %4.2f]", utm.easting-t.at<double>(0), utm.northing-t.at<double>(1), utm.altitude-t.at<double>(2));
}

PoseEstimationIO::PoseEstimationIO(PoseEstimation* stage, double rate, bool do_delay_keyframes)
    : WorkerThreadBase("Publisher [pose_estimation]", static_cast<int64_t>(1/rate*1000.0), true),
      _is_time_ref_set(false),
      _is_new_output_path_set(false),
      _do_delay_keyframes(do_delay_keyframes),
      _t_ref({0, 0}),
      _stage_handle(stage)
{
  if (!_stage_handle)
    throw(std::invalid_argument("Error: Could not create PoseEstimationIO. Stage handle points to NULL."));
}

void PoseEstimationIO::setOutputPath(const std::string &path)
{
  _path_output = path;
  _is_new_output_path_set = true;
}

void PoseEstimationIO::initLog(const std::string &filepath)
{
  loguru::add_file((filepath + "/publisher.log").c_str(), loguru::Append, loguru::Verbosity_MAX);

  LOG_F(INFO, "Successfully initialized %s publisher!", _stage_handle->_stage_name.c_str());
}

bool PoseEstimationIO::process()
{
  if (_is_new_output_path_set)
  {
    initLog(_path_output);
    _is_new_output_path_set = false;
  }

  if (!_stage_handle->_buffer_do_publish.empty())
  {
    // Grab frame from pose estimation geoereferenced mmts
    Frame::Ptr frame = _stage_handle->getNewFramePublish();

    // Data to be published for every georeferenced frame (usually small data packages).
    // Poses get only published, if suppress flag was not set (old poses might crash state estimate filter)
    if (!(_stage_handle->_do_suppress_outdated_pose_pub && _stage_handle->_buffer_do_publish.size() > 1))
      publishPose(frame);

    // Keyframes to be published (big data packages -> publish only if needed)
    if ((_stage_handle->_use_fallback && !frame->hasAccuratePose() && _stage_handle->estimatePercOverlap(frame) < _stage_handle->_overlap_max_fallback)
         ||(!_stage_handle->_use_vslam && _stage_handle->estimatePercOverlap(frame) < _stage_handle->_overlap_max_fallback)
         || (frame->isKeyframe() && _stage_handle->estimatePercOverlap(frame) < _stage_handle->_overlap_max))
    {
      _stage_handle->updatePreviousRoi(frame);
      if (_do_delay_keyframes)
        scheduleFrame(frame);
      else
        publishFrame(frame);
    }
  }
  if (!_stage_handle->_img_debug.empty())
  {
    std::unique_lock<std::mutex> lock(_stage_handle->_mutex_img_debug);
    _stage_handle->_transport_img(_stage_handle->_img_debug, "debug/tracked");
    _stage_handle->_img_debug.release();
  }
  publishScheduled();
  return false;
}

void PoseEstimationIO::reset()
{
  std::unique_lock<std::mutex> lock(_mutex_reset_requested);
  _is_time_ref_set = false;
  _t_ref = TimeReference{0, 0};
  _reset_requested = false;
}

void PoseEstimationIO::publishPose(const Frame::Ptr &frame)
{
  LOG_F(INFO, "Publishing pose of frame #%u...", frame->getFrameId());

  // Save trajectories
  if (_stage_handle->_settings_save.save_trajectory_gnss || _stage_handle->_settings_save.save_trajectory_visual)
    io::saveTimestamp(frame->getTimestamp(), frame->getFrameId(), _stage_handle->_stage_path + "/trajectory/timestamps.txt");
  if (_stage_handle->_settings_save.save_trajectory_gnss)
    io::saveTrajectory(frame->getTimestamp(), frame->getDefaultPose(), _stage_handle->_stage_path + "/trajectory/gnss_traj_TUM.txt");
  if (_stage_handle->_settings_save.save_trajectory_visual && frame->hasAccuratePose())
    io::saveTrajectory(frame->getTimestamp(), frame->getPose(), _stage_handle->_stage_path + "/trajectory/f_traj_TUM.txt");
  if (_stage_handle->_settings_save.save_trajectory_visual && frame->isKeyframe())
    io::saveTrajectory(frame->getTimestamp(), frame->getPose(), _stage_handle->_stage_path + "/trajectory/kf_traj_TUM.txt");

  _stage_handle->_transport_pose(frame->getPose(), frame->getGnssUtm().zone, frame->getGnssUtm().band, "output/pose/visual");
}

void PoseEstimationIO::publishSparseCloud(const Frame::Ptr &frame)
{
  cv::Mat sparse_cloud = frame->getSparseCloud();
  if (!sparse_cloud.empty())
    _stage_handle->_transport_pointcloud(sparse_cloud, "output/pointcloud");
}

void PoseEstimationIO::publishFrame(const Frame::Ptr &frame)
{
  // First update statistics about outgoing frame rate
  _stage_handle->updateFpsStatisticsOutgoing();

  // Two situation can occure, when publishing a frame is triggered
  // 1) Frame is marked as keyframe by the SLAM -> publish directly
  // 2) Frame is not marked as keyframe -> mostly in GNSS only situations.
  LOG_IF_F(INFO, frame->isKeyframe(), "Publishing keyframe #%u...", frame->getFrameId());
  LOG_IF_F(INFO, !frame->isKeyframe(), "Publishing frame #%u...", frame->getFrameId());

  publishSparseCloud(frame);

  _stage_handle->_transport_frame(frame, "output/frame");
  _stage_handle->printGeoReferenceInfo(frame);

  // Save image related data
  if (_stage_handle->_settings_save.save_frames && !frame->isKeyframe())
    io::saveExifImage(frame, _stage_handle->_stage_path + "/frames", "frames", frame->getFrameId(), true);
  if (_stage_handle->_settings_save.save_keyframes && frame->isKeyframe())
    io::saveExifImage(frame, _stage_handle->_stage_path + "/keyframes", "keyframe", frame->getFrameId(), true);
  if (_stage_handle->_settings_save.save_keyframes_full && frame->isKeyframe())
    io::saveExifImage(frame, _stage_handle->_stage_path + "/keyframes_full", "keyframe_full", frame->getFrameId(), false);
}

void PoseEstimationIO::scheduleFrame(const Frame::Ptr &frame)
{
  long t_world = getCurrentTimeMilliseconds();       // millisec
  uint64_t t_frame = frame->getTimestamp()/1000000;  // millisec

  // Check if either first measurement ever (does not have to be keyframe)
  // Or if first keyframe
  if ((frame->isKeyframe() && !_is_time_ref_set)
      || (_t_ref.first == 0 && _t_ref.second == 0))
  {
    _t_ref.first = t_world;
    _t_ref.second = t_frame;
    if (frame->isKeyframe())
      _is_time_ref_set = true;
  }

  // Create task for publish
  uint64_t dt = (t_frame - _t_ref.second);
  std::unique_lock<std::mutex> lock(_mutex_schedule);
  _schedule.emplace_back(Task{(long)dt, frame});

  // Time until schedule
  long t_remain = ((long)dt) - (getCurrentTimeMilliseconds()-_t_ref.first);
  LOG_F(INFO, "Scheduled publish frame #%u in %4.2fs", frame->getFrameId(), (double)t_remain/1000);
}

void PoseEstimationIO::publishScheduled()
{
  if (_schedule.empty())
    return;
  Task task = _schedule.front();
  if (task.first < (getCurrentTimeMilliseconds() - _t_ref.first))
  {
    _mutex_schedule.lock();
    _schedule.pop_front();
    _mutex_schedule.unlock();
    publishFrame(task.second);
  }
}

void PoseEstimationIO::publishAll()
{
  _mutex_schedule.lock();
  for (const auto &task : _schedule)
  {
    publishFrame(task.second);
  }
  _schedule.clear();
  _mutex_schedule.unlock();
}
