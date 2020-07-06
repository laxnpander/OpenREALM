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

#include <openvslam/system.h>
#include <openvslam/type.h>
#include <openvslam/data/keyframe.h>
#include <yaml-cpp/yaml.h>

namespace realm
{

class OpenVslamFramePublisher;

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

  cv::Mat getLastDrawnFrame() const;
  cv::Mat invertPose(const cv::Mat &pose) const;
  cv::Mat convertToCv(const openvslam::Mat44_t &mat) const;
};

} // namespace realm

#endif //OPENREALM_OPEN_VSLAM_H

