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

#ifndef OPENREALM_OPEN_VSLAM_H
#define OPENREALM_OPEN_VSLAM_H

#include <realm_vslam_base/visual_slam_IF.h>
#include <realm_vslam_base/visual_slam_settings.h>
#include <realm_core/camera_settings.h>
#include <realm_core/worker_thread_base.h>

#include <openvslam/system.h>
#include <openvslam/type.h>
#include <openvslam/data/keyframe.h>
#include <yaml-cpp/yaml.h>

namespace realm
{

class OpenVslamKeyframeUpdater;

class OpenVslam : public VisualSlamIF
{
public:
  OpenVslam(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set);
  State track(Frame::Ptr &frame, const cv::Mat &T_c2w_initial) override;
  void close() override;
  void reset() override;
  void printSettingsToLog() override;

  bool drawTrackedImage(cv::Mat &img) const override;

  cv::Mat getTrackedMapPoints() const override;

private:

  unsigned int _nrof_keyframes;

  double _resizing;

  std::string _path_vocabulary;

  mutable std::mutex _mutex_last_drawn_frame;
  cv::Mat _last_drawn_frame;

  mutable std::mutex _mutex_last_keyframe;
  openvslam::data::keyframe* _last_keyframe;

  std::shared_ptr<openvslam::system> _vslam;
  std::shared_ptr<openvslam::config> _config;
  std::shared_ptr<openvslam::publish::frame_publisher> _frame_publisher;
  std::shared_ptr<openvslam::publish::map_publisher> _map_publisher;
  std::unique_ptr<OpenVslamKeyframeUpdater> _keyframe_updater;

  cv::Mat getLastDrawnFrame() const;
  cv::Mat invertPose(const cv::Mat &pose) const;
  cv::Mat convertToCv(const openvslam::Mat44_t &mat) const;
};

class OpenVslamKeyframeUpdater : public WorkerThreadBase
{
public:
  OpenVslamKeyframeUpdater(const std::string &thread_name, int64_t sleep_time, bool verbose);

  void add(const std::weak_ptr<Frame> &frame_realm, openvslam::data::keyframe* frame_vslam);

private:

  /// Container for OpenREALM frames to corresponding OpenVSLAM keyframe
  std::list<std::pair<std::weak_ptr<Frame>, openvslam::data::keyframe*>> _keyframe_links;

  bool process() override;

  void reset() override;

};

} // namespace realm

#endif //OPENREALM_OPEN_VSLAM_H

