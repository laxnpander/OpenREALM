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

#include <realm_stages/pose_estimation.h>

using namespace realm;
using namespace stages;

PoseEstimation::PoseEstimation(const StageSettings::Ptr &stage_set,
                               const VisualSlamSettings::Ptr &vslam_set,
                               const CameraSettings::Ptr &cam_set)
    : StageBase("pose_estimation", stage_set->get<std::string>("path_output"), stage_set->get<int>("queue_size")),
      _is_georef_initialized(false),
      _use_vslam(stage_set->get<int>("use_vslam") > 0),
      _use_fallback(stage_set->get<int>("use_fallback") > 0),
      _do_update_georef(stage_set->get<int>("update_georef") > 0),
      _do_suppress_outdated_pose_pub(stage_set->get<int>("suppress_outdated_pose_pub") > 0),
      _th_error_georef(stage_set->get<double>("th_error_georef")),
      _overlap_max(stage_set->get<double>("overlap_max")),
      _overlap_max_fallback(stage_set->get<double>("overlap_max_fallback")),
      _settings_save({stage_set->get<int>("save_trajectory_gnss") > 0,
                      stage_set->get<int>("save_trajectory_visual") > 0,
                      stage_set->get<int>("save_frames") > 0,
                      stage_set->get<int>("save_keyframes") > 0,
                      stage_set->get<int>("save_keyframes_full") > 0})
{
  if (_use_vslam)
  {
    _vslam = VisualSlamFactory::create(vslam_set, cam_set);

    // Set reset callback from vSLAM to this node
    // therefore if SLAM resets itself, node is being informed
    std::function<void(void)> reset_func = std::bind(&PoseEstimation::reset, this);
    _vslam->RegisterResetCallback(reset_func);

    // Set pose update callback
    namespace ph = std::placeholders;
    VisualSlamIF::PoseUpdateFuncCb update_func = std::bind(&PoseEstimation::updateKeyframeCb, this, ph::_1, ph::_2, ph::_3);
    _vslam->RegisterUpdateTransport(update_func);

    // Create geo reference initializer
    _georef = std::make_shared<GeometricReferencer>(_th_error_georef);
  }

  // Create Pose Estimation publisher
  _stage_publisher.reset(new PoseEstimationIO(this, true));
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

void PoseEstimation::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateFpsStatisticsIncoming();

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
  // Trigger; true if during this processing something
  // has been sucessfully processed.
  bool has_processed = false;

  // Grab georeference flag once at the beginning, to avoid multithread problems
  if (_use_vslam)
    _is_georef_initialized = _georef->isInitialized();

  // Process new frames without a visual pose currently
  if (!_buffer_no_pose.empty())
  {
    // Grab frame from buffer with no poses
    Frame::Ptr frame = getNewFrameTracking();

    LOG_F(INFO, "Processing frame #%i with timestamp %lu!", frame->getFrameId(), frame->getTimestamp());

    // Track current frame -> compute visual accurate pose
    track(frame);

    // Identify buffer for push
    if (frame->hasAccuratePose() && _is_georef_initialized)
    {
      if (_do_update_georef && !_georef->isBuisy())
      {
        std::thread t(std::bind(&GeospatialReferencerIF::update, _georef, std::make_shared<Frame>(*frame)));
        t.detach();
      }
      pushToBufferAll(frame);
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
  if (_use_vslam)
  {
    if (!_is_georef_initialized && !_buffer_pose_init.empty() && !_georef->isBuisy())
    {
      // Branch: Georef is not calculated yet
      std::thread t(std::bind(&GeospatialReferencerIF::init, _georef, _buffer_pose_init));
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

  // Compute visual pose
  _mutex_vslam.lock();
  VisualSlamIF::State state = _vslam->Track(frame);
  _mutex_vslam.unlock();

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
  _vslam->DrawTrackedImage(_img_debug);
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
    _vslam->Reset();
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
    _georef.reset(new GeometricReferencer(_th_error_georef));
  _stage_publisher->requestReset();
  _is_georef_initialized = false;
  _reset_requested = false;
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
  _roi_prev = estimateProjectedRoi(frame);
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
        frame->setSurfacePoints(points);
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
  cv::Rect2d roi_curr = estimateProjectedRoi(frame);

  if (roi_curr.x + roi_curr.width < _roi_prev.x
      || roi_curr.x > _roi_prev.x + _roi_prev.width
      || roi_curr.y - roi_curr.height > _roi_prev.y
      || roi_curr.y < _roi_prev.y - _roi_prev.y - _roi_prev.height)
    return 0.0;

  // Assume full roi overlap
  cv::Point2d pt_ulc(roi_curr.x, roi_curr.y);
  cv::Point2d pt_lrc(roi_curr.x + roi_curr.width, roi_curr.y - roi_curr.height);

  // Adjust the roi to the overlap
  if (roi_curr.x < _roi_prev.x)
    pt_ulc.x = _roi_prev.x;
  if (roi_curr.x+roi_curr.width > _roi_prev.x+_roi_prev.width)
    pt_lrc.x = _roi_prev.x+_roi_prev.width;
  if (roi_curr.y > _roi_prev.y)
    pt_ulc.y = _roi_prev.y;
  if (roi_curr.y-roi_curr.height < _roi_prev.y-_roi_prev.height)
    pt_lrc.y = _roi_prev.y-_roi_prev.height;

  // create world frame roi
  cv::Rect2d roi_overlap(pt_ulc.x, pt_ulc.y, pt_lrc.x-pt_ulc.x, pt_ulc.y-pt_lrc.y);

  return (roi_overlap.area() / roi_curr.area())*100;
}

cv::Rect2d PoseEstimation::estimateProjectedRoi(const Frame::Ptr &frame)
{
  camera::Pinhole cam = frame->getCamera();
  cam.setPose(frame->getPose());
  return cam.projectImageBoundsToPlaneRoi(_plane_ref.pt, _plane_ref.n);
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
  // Apply estimated geo reference to all measurements in the buffer
  while(!_buffer_pose_all.empty())
  {
    _mutex_buffer_pose_all.lock();
    Frame::Ptr frame = _buffer_pose_all.front();
    _buffer_pose_all.pop_front();
    _mutex_buffer_pose_all.unlock();

    _mutex_t_w2g.lock();
    _T_w2g = _georef->getTransformation();
    _mutex_t_w2g.unlock();

    // But check first, if frame has actually a visually estimated pose information
    // In case of default GNSS pose generated from lat/lon/alt/heading, pose is already in world frame
    if (frame->hasAccuratePose())
      frame->applyGeoreference(_T_w2g);
    pushToBufferPublish(frame);
  }
}

void PoseEstimation::printGeoReferenceInfo(const Frame::Ptr &frame)
{
  UTMPose utm = frame->getGnssUtm();
  camera::Pinhole cam = frame->getCamera();
  cam.setPose(frame->getPose());
  cv::Mat t = cam.t();

  LOG_F(INFO, "Georeferenced pose:");
  LOG_F(INFO, "GNSS: [%10.2f, %10.2f, %4.2f]", utm.easting, utm.northing, utm.altitude);
  LOG_F(INFO, "GNSS: [%10.2f, %10.2f, %4.2f]", t.at<double>(0), t.at<double>(1), t.at<double>(2));
  LOG_F(INFO, "Diff: [%10.2f, %10.2f, %4.2f]", utm.easting-t.at<double>(0), utm.northing-t.at<double>(1), utm.altitude-t.at<double>(2));
}

PoseEstimationIO::PoseEstimationIO(PoseEstimation* stage, bool do_delay_keyframes)
    : WorkerThreadBase("Publisher [pose_estimation]", true),
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
    io::saveTimestamp(frame->getTimestamp(), frame->getFrameId(), _stage_handle->_stage_path + "/trajectory", "timestamps");
  if (_stage_handle->_settings_save.save_trajectory_gnss)
    io::saveTrajectory(frame->getTimestamp(), frame->getDefaultPose(), _stage_handle->_stage_path + "/trajectory", "gnss_traj_TUM");
  if (_stage_handle->_settings_save.save_trajectory_visual && frame->hasAccuratePose())
    io::saveTrajectory(frame->getTimestamp(), frame->getPose(), _stage_handle->_stage_path + "/trajectory", "f_traj_TUM");
  if (_stage_handle->_settings_save.save_trajectory_visual && frame->isKeyframe())
    io::saveTrajectory(frame->getTimestamp(), frame->getPose(), _stage_handle->_stage_path + "/trajectory", "kf_traj_TUM");

  _stage_handle->_transport_pose(frame->getPose(), frame->getGnssUtm().zone, frame->getGnssUtm().band, "output/pose/visual");
}

void PoseEstimationIO::publishSurfacePoints(const Frame::Ptr &frame)
{
  cv::Mat surface_pts = frame->getSurfacePoints();
  if (!surface_pts.empty())
    _stage_handle->_transport_pointcloud(surface_pts, "output/pointcloud");
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

  publishSurfacePoints(frame);

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
