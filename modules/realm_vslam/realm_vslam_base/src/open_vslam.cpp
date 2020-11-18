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

#include <realm_vslam_base/open_vslam.h>

#include <openvslam/config.h>
#include <openvslam/data/landmark.h>
#include <openvslam/publish/map_publisher.h>
#include <openvslam/publish/frame_publisher.h>

using namespace realm;

OpenVslam::OpenVslam(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set)
 : _nrof_keyframes(0),
   _last_keyframe(nullptr),
   _resizing((*vslam_set)["resizing"].toDouble()),
   _path_vocabulary((*vslam_set)["path_vocabulary"].toString()),
   _keyframe_updater(new OpenVslamKeyframeUpdater("Keyframe Updater", 100, false))
{
  // ov: OpenVSLAM
  // or: OpenREALM
  YAML::Node settings;
  settings["Camera.name"] = "cam";
  settings["Camera.setup"] = "monocular";
  settings["Camera.model"] = "perspective";
  settings["Camera.color_order"] = "RGB";
  settings["Camera.fx"] = (*cam_set)["fx"].toDouble() * _resizing;
  settings["Camera.fy"] = (*cam_set)["fy"].toDouble() * _resizing;
  settings["Camera.cx"] = (*cam_set)["cx"].toDouble() * _resizing;
  settings["Camera.cy"] = (*cam_set)["cy"].toDouble() * _resizing;
  settings["Camera.k1"] = (*cam_set)["k1"].toDouble();
  settings["Camera.k2"] = (*cam_set)["k2"].toDouble();
  settings["Camera.p1"] = (*cam_set)["p1"].toDouble();
  settings["Camera.p2"] = (*cam_set)["p2"].toDouble();
  settings["Camera.k3"] = (*cam_set)["k3"].toDouble();
  settings["Camera.fps"] = (*cam_set)["fps"].toDouble();
  settings["Camera.cols"] = (*cam_set)["width"].toInt() * _resizing;
  settings["Camera.rows"] = (*cam_set)["height"].toInt() * _resizing;
  settings["Feature.max_num_keypoints"] = (*vslam_set)["nrof_features"].toInt();
  settings["Feature.scale_factor"] = (*vslam_set)["scale_factor"].toFloat();
  settings["Feature.ini_fast_threshold"] = (*vslam_set)["ini_th_FAST"].toInt();
  settings["Feature.min_fast_threshold"] = (*vslam_set)["min_th_FAST"].toInt();

  _config = std::make_shared<openvslam::config>(settings, "");
  _vslam = std::make_shared<openvslam::system>(_config, _path_vocabulary);
  _frame_publisher = _vslam->get_frame_publisher();
  _map_publisher = _vslam->get_map_publisher();

  _vslam->startup();
  _keyframe_updater->start();
}

VisualSlamIF::State OpenVslam::track(Frame::Ptr &frame, const cv::Mat &T_c2w_initial)
{
  // Set image resizing accoring to settings
  frame->setImageResizeFactor(_resizing);

  // ORB SLAM returns a transformation from the world to the camera frame (T_w2c). In case we provide an initial guess
  // of the current pose, we have to invert this before, because in OpenREALM the standard is defined as T_c2w.
  cv::Mat T_w2c;
  openvslam::Mat44_t T_w2c_eigen;
  if (T_c2w_initial.empty())
  {
    T_w2c_eigen = _vslam->feed_monocular_frame(frame->getResizedImageRaw(), frame->getTimestamp()*10e-9);
    T_w2c = convertToCv(T_w2c_eigen);
  }
  else
  {
    // prior not yet implemented
  }

  openvslam::tracker_state_t tracker_state = _vslam->get_tracker_state();

  // Draw frame of tracked features
  _mutex_last_drawn_frame.lock();
  _last_drawn_frame = _frame_publisher->draw_frame();
  _mutex_last_drawn_frame.unlock();

  // In case tracking was successfull and slam not lost
  if (tracker_state == openvslam::tracker_state_t::Tracking)
  {
    // Get list of keyframes
    std::vector<openvslam::data::keyframe*> keyframes;
    unsigned int current_nrof_keyframes = _map_publisher->get_keyframes(keyframes);

    // Not ideal implementation, but I am not sure that the keyframes are sorted
    _mutex_last_keyframe.lock();
    if (_last_keyframe == nullptr)
      _last_keyframe = keyframes.back();
    else
    {
      for (auto kf : keyframes)
        if (kf->id_ > _last_keyframe->id_)
          _last_keyframe = kf;
    }
    _mutex_last_keyframe.unlock();

    // Pose definition as 3x4 matrix, calculated as 4x4 with last row (0, 0, 0, 1)
    // OpenVSLAM pose is defined as T_w2c, however the more intuitive way to describe
    // it for mapping is T_c2w (camera to world) therefore invert the pose matrix
    cv::Mat T_c2w = invertPose(T_w2c);

    // Remove last row of 0,0,0,1
    T_c2w.pop_back();
    frame->setVisualPose(T_c2w);

    cv::Mat surface_pts = getTrackedMapPoints();
    frame->setSparseCloud(surface_pts, true);

    // Check current state of the slam
    if (_nrof_keyframes == 0 && current_nrof_keyframes > 0)
    {
      _nrof_keyframes = current_nrof_keyframes;
      return State::INITIALIZED;
    }
    else if (current_nrof_keyframes != _nrof_keyframes)
    {
      // We want to keep all keyframes once created
      _last_keyframe->set_not_to_be_erased();

      // Pass to keyframe updater, which regularly checks if points or pose have changed
      _keyframe_updater->add(frame, _last_keyframe);

      _nrof_keyframes = current_nrof_keyframes;
      return State::KEYFRAME_INSERT;
    }
    else
    {
      return State::FRAME_INSERT;
    }
  }

  return State::LOST;
}

void OpenVslam::close()
{
  _vslam->shutdown();
  _keyframe_updater->requestFinish();
  _keyframe_updater->join();
}

void OpenVslam::reset()
{
  LOG_F(INFO, "Reseting visual SLAM...");
  _vslam->request_reset();
  _keyframe_updater->requestReset();

  std::lock_guard<std::mutex> lock(_mutex_last_keyframe);
  _last_keyframe = nullptr;
  _nrof_keyframes = 0;
  LOG_F(INFO, "Finished reseting visual SLAM.");
}

cv::Mat OpenVslam::getTrackedMapPoints() const
{
  _mutex_last_keyframe.lock();
  std::vector<openvslam::data::landmark*> landmarks = _last_keyframe->get_landmarks();
  _mutex_last_keyframe.unlock();

  cv::Mat points;
  points.reserve(landmarks.size());
  for (const auto &lm : landmarks)
  {
    if (!lm || lm->will_be_erased())
    {
      continue;
    }
    openvslam::Vec3_t pos = lm->get_pos_in_world();
    cv::Mat pt = (cv::Mat_<double>(1, 3) << pos[0], pos[1], pos[2]);
    points.push_back(pt);
  }
  return points;
}

bool OpenVslam::drawTrackedImage(cv::Mat &img) const
{
  img = getLastDrawnFrame();
  return !img.empty();
}

cv::Mat OpenVslam::getLastDrawnFrame() const
{
  std::lock_guard<std::mutex> lock(_mutex_last_drawn_frame);
  return _last_drawn_frame.clone();
}

cv::Mat OpenVslam::invertPose(const cv::Mat &pose) const
{
  cv::Mat pose_inv = cv::Mat::eye(4, 4, pose.type());
  cv::Mat R_t = (pose.rowRange(0, 3).colRange(0, 3)).t();
  cv::Mat t = -R_t*pose.rowRange(0, 3).col(3);
  R_t.copyTo(pose_inv.rowRange(0, 3).colRange(0, 3));
  t.copyTo(pose_inv.rowRange(0, 3).col(3));
  return pose_inv;
}

cv::Mat OpenVslam::convertToCv(const openvslam::Mat44_t &mat_eigen) const
{
  cv::Mat mat_cv = (cv::Mat_<double>(4, 4) <<
      mat_eigen(0, 0), mat_eigen(0, 1), mat_eigen(0, 2), mat_eigen(0, 3),
      mat_eigen(1, 0), mat_eigen(1, 1), mat_eigen(1, 2), mat_eigen(1, 3),
      mat_eigen(2, 0), mat_eigen(2, 1), mat_eigen(2, 2), mat_eigen(2, 3),
      mat_eigen(3, 0), mat_eigen(3, 1), mat_eigen(3, 2), mat_eigen(3, 3)
      );
  return mat_cv;
}

void OpenVslam::printSettingsToLog()
{

}

void OpenVslam::queueImuData(const VisualSlamIF::ImuData &imu)
{
  // TBD
}

OpenVslamKeyframeUpdater::OpenVslamKeyframeUpdater(const std::string &thread_name, int64_t sleep_time, bool verbose)
  : WorkerThreadBase(thread_name, sleep_time, verbose)
{
}

void OpenVslamKeyframeUpdater::add(const std::weak_ptr<Frame> &frame_realm, openvslam::data::keyframe *frame_vslam)
{
  // We create a connection between the OpenREALM frames and the OpenVSLAM keyframes, so we can update points and
  // poses easily
  _keyframe_links.emplace_back(std::make_pair(frame_realm, frame_vslam));
}

bool OpenVslamKeyframeUpdater::process()
{
  bool has_processed = false;

  for (auto it = _keyframe_links.begin(); it != _keyframe_links.end(); )
  {
    std::shared_ptr<Frame> frame_realm = it->first.lock();
    openvslam::data::keyframe* frame_slam = it->second;

    // The question now is, if the weak pointers in the link queue are still pointing to existing objects
    // If yes, we have no problem of setting the surface points in that frame.
    // If no, we have to delete the pair from the dequeue to avoid unnecessary computations in the future
    if (frame_realm != nullptr && frame_slam != nullptr && !frame_slam->will_be_erased())
    {
      // This is to prevent a racing condition, when the frame is already added in the updater, but pose has not
      // yet been set. The "accurate pose" flag is thread safe.
      if (!frame_realm->hasAccuratePose())
        continue;

      // Frame is still in the memory
      // Therefore update point cloud
      cv::Mat surface_points = frame_realm->getSparseCloud();
      std::vector<openvslam::data::landmark*> landmarks = frame_slam->get_landmarks();

      cv::Mat new_surface_points;
      new_surface_points.reserve(landmarks.size());

      for (const auto &lm : landmarks)
      {
        if (!lm || lm->will_be_erased())
        {
          continue;
        }
        openvslam::Vec3_t pos = lm->get_pos_in_world();
        cv::Mat pt = (cv::Mat_<double>(1, 3) << pos[0], pos[1], pos[2]);
        new_surface_points.push_back(pt);
      }

      if (surface_points.rows != new_surface_points.rows)
      {
        LOG_IF_F(INFO, m_verbose, "Updating frame %u: %u --> %u", frame_realm->getFrameId(), surface_points.rows, new_surface_points.rows);
        frame_realm->setSparseCloud(new_surface_points, true);
      }

      has_processed = true;
    }
    else
    {
      LOG_IF_F(INFO, m_verbose, "Frame object out of scope. Deleting reference.");
      // Frame is not existing anymore, delete from dequeue
      it = _keyframe_links.erase(it);
    }
  }
  return has_processed;
}

void OpenVslamKeyframeUpdater::reset()
{
  _keyframe_links.clear();
}