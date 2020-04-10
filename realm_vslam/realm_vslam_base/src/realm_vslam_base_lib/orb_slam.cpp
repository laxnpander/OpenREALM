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

#include "realm_vslam_base/orb_slam.h"

using namespace realm;

OrbSlam2::OrbSlam2(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set)
: _prev_keyid(-1),
  _resizing(vslam_set->get<double>("resizing")),
  _path_vocabulary(vslam_set->get<std::string>("path_vocabulary"))
{
  // Read the settings files
  cv::Mat K_32f = cv::Mat::eye(3, 3, CV_32F);
  K_32f.at<float>(0, 0) = (float)(cam_set->get<double>("fx") * _resizing);
  K_32f.at<float>(1, 1) = (float)(cam_set->get<double>("fy") * _resizing);
  K_32f.at<float>(0, 2) = (float)(cam_set->get<double>("cx") * _resizing);
  K_32f.at<float>(1, 2) = (float)(cam_set->get<double>("cy") * _resizing);

  cv::Mat dist_coeffs_32f = cv::Mat::zeros(1, 5, CV_32F);
  dist_coeffs_32f.at<float>(0) = (float)cam_set->get<double>("k1");
  dist_coeffs_32f.at<float>(1) = (float)cam_set->get<double>("k2");
  dist_coeffs_32f.at<float>(2) = (float)cam_set->get<double>("p1");
  dist_coeffs_32f.at<float>(3) = (float)cam_set->get<double>("p2");
  dist_coeffs_32f.at<float>(4) = (float)cam_set->get<double>("k3");

  _cam_set.calibration = K_32f;
  _cam_set.distortion = dist_coeffs_32f;
  _cam_set.fps = (float)cam_set->get<double>("fps");
  _cam_set.imgWidth = cam_set->get<int>("width");
  _cam_set.imgHeight = cam_set->get<int>("height");
  _cam_set.rgb = 0;

  _track_set.nFeatures = vslam_set->get<int>("nrof_features");
  _track_set.nLevels = vslam_set->get<int>("n_pyr_levels");
  _track_set.scaleFactor = (float)vslam_set->get<double>("scale_factor");
  _track_set.iniThFast = vslam_set->get<int>("ini_th_FAST");
  _track_set.minThFast = vslam_set->get<int>("min_th_FAST");

  _slam = new ORB_SLAM2::System(_cam_set, _track_set, _view_set, _path_vocabulary, ORB_SLAM2::System::MONOCULAR, false);

  namespace ph = std::placeholders;
  std::function<void(ORB_SLAM2::KeyFrame*)> kf_update = std::bind(&OrbSlam2::keyframeUpdateCb, this, ph::_1);
  _slam->RegisterKeyTransport(kf_update);
}

OrbSlam2::~OrbSlam2()
{
  _slam->Shutdown();
  delete _slam;
}

VisualSlamIF::State OrbSlam2::Track(Frame::Ptr &frame)
{
  // Set image resizing accoring to settings
  frame->setImageResizeFactor(_resizing);

  // T_w2c defined as transformation from world to camera frame
  cv::Mat T_w2c;
  T_w2c = _slam->TrackMonocular(frame->getResizedImageRaw(), frame->getTimestamp());

  // In case tracking was successfull and slam not lost
  if (!T_w2c.empty())
  {
    // Pose definition as 3x4 matrix, calculated as 4x4 with last row (0, 0, 0, 1)
    // ORB SLAM 2 pose is defined as T_w2c, however the more intuitive way to describe
    // it for mapping is T_c2w (camera to world) therefore invert the pose matrix
    cv::Mat T_c2w = invertPose(T_w2c);

    // Also convert to double precision
    T_c2w.convertTo(T_c2w, CV_64F);
    T_c2w.pop_back();
    frame->setVisualPose(T_c2w);

    cv::Mat surface_pts = GetTrackedMapPoints();
    frame->setSurfacePoints(surface_pts);

    // Check if new frame is keyframe by comparing current keyid with last keyid
    int32_t keyid = (int32_t)_slam->GetLastKeyFrameId();

    // Check current state of the slam
    if (_prev_keyid == -1)
    {
      _prev_keyid = keyid;
      return State::INITIALIZED;
    }
    else if (_prev_keyid != keyid)
    {
      _prev_keyid = keyid;
      _orb_to_frame_ids.insert({keyid, frame->getFrameId()});
      return State::KEYFRAME_INSERT;
    }
    else
    {
      return State::FRAME_INSERT;
    }
  }
  return State::LOST;
}

void OrbSlam2::Reset()
{
  _slam->Reset();
}

void OrbSlam2::Close()
{
  _slam->Shutdown();
}

cv::Mat OrbSlam2::GetTrackedMapPoints() const
{
  vector<ORB_SLAM2::MapPoint *> mappoints;

  size_t n = 0;
  mappoints = _slam->GetTrackedMapPoints();
  n = mappoints.size();

  cv::Mat cvpoints;
  cvpoints.reserve(n);
  for (size_t i = 0; i < n; ++i)
  {
    if (mappoints[i] != nullptr)
    {
      cv::Mat p = mappoints[i]->GetWorldPos().t();
      cvpoints.push_back(p);
    }
  }
  cvpoints.convertTo(cvpoints, CV_64F);
  return cvpoints;
}

cv::Mat OrbSlam2::GetMapPoints() const
{

}

bool OrbSlam2::DrawTrackedImage(cv::Mat &img) const
{
  img = _slam->DrawTrackedImage();
  return true;
}

void OrbSlam2::RegisterUpdateTransport(const VisualSlamIF::PoseUpdateFuncCb &func)
{
  _pose_update_func_cb = func;
}

void OrbSlam2::RegisterResetCallback(const VisualSlamIF::ResetFuncCb &func)
{
  if (func)
  {
    _slam->RegisterResetCallback(func);
  }
}

void OrbSlam2::keyframeUpdateCb(ORB_SLAM2::KeyFrame* kf)
{
  if (kf != nullptr && _pose_update_func_cb)
  {
    auto id = (uint32_t) kf->mnFrameId;

    // Get update on pose
    cv::Mat T_w2c = kf->GetPose();
    cv::Mat T_c2w = invertPose(T_w2c);
    T_c2w.convertTo(T_c2w, CV_64F);
    T_c2w.pop_back();

    // Get update on map points
    std::set<ORB_SLAM2::MapPoint*> map_points = kf->GetMapPoints();
    cv::Mat points;
    points.reserve(map_points.size());
    for (const auto &pt : map_points)
      if (pt != nullptr)
        points.push_back(pt->GetWorldPos().t());
    points.convertTo(points, CV_64F);

    // Transport to update function
    _pose_update_func_cb(_orb_to_frame_ids[id], T_c2w, points);
  }
}

cv::Mat OrbSlam2::invertPose(const cv::Mat &pose) const
{
  cv::Mat pose_inv = cv::Mat::eye(4, 4, pose.type());
  cv::Mat R_t = (pose.rowRange(0, 3).colRange(0, 3)).t();
  cv::Mat t = -R_t*pose.rowRange(0, 3).col(3);
  R_t.copyTo(pose_inv.rowRange(0, 3).colRange(0, 3));
  t.copyTo(pose_inv.rowRange(0, 3).col(3));
  return pose_inv;
}

void OrbSlam2::printSettingsToLog()
{
  LOG_F(INFO, "### OrbSlam2 general settings ###");
  LOG_F(INFO, "- use_viewer: %i", _use_viewer);
  LOG_F(INFO, "- resizing: %4.2f", _resizing);
  LOG_F(INFO, "- path settings: %s", _path_settings.c_str());
  LOG_F(INFO, "- path vocabulary: %s", _path_vocabulary.c_str());
  LOG_F(INFO, "### OrbSlam2 camera settings ###");
  LOG_F(INFO, "- fps: %4.2f", _cam_set.fps);
  LOG_F(INFO, "- rgb: %i", _cam_set.rgb);
  LOG_F(INFO, "- width: %i", _cam_set.imgWidth);
  LOG_F(INFO, "- height: %i", _cam_set.imgHeight);
  LOG_F(INFO, "- fx: %4.2f", _cam_set.calibration.at<float>(0, 0));
  LOG_F(INFO, "- fy: %4.2f", _cam_set.calibration.at<float>(1, 1));
  LOG_F(INFO, "- cx: %4.2f", _cam_set.calibration.at<float>(0, 2));
  LOG_F(INFO, "- cy: %4.2f", _cam_set.calibration.at<float>(1, 2));
  LOG_F(INFO, "- k1: %2.6f", _cam_set.distortion.at<float>(0));
  LOG_F(INFO, "- k2: %2.6f", _cam_set.distortion.at<float>(1));
  LOG_F(INFO, "- p1: %2.6f", _cam_set.distortion.at<float>(2));
  LOG_F(INFO, "- p2: %2.6f", _cam_set.distortion.at<float>(3));
  LOG_F(INFO, "- k3: %2.6f", _cam_set.distortion.at<float>(4));
  LOG_F(INFO, "### OrbSlam2 SLAM settings ###");
  LOG_F(INFO, "- nFeatures: %i", _track_set.nFeatures);
  LOG_F(INFO, "- nLevels: %i", _track_set.nLevels);
  LOG_F(INFO, "- scaleFactor: %4.2f", _track_set.scaleFactor);
  LOG_F(INFO, "- iniThFast: %i", _track_set.iniThFast);
  LOG_F(INFO, "- minThFast: %i", _track_set.minThFast);
}