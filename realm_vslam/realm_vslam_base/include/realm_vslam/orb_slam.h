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

#pragma once

#include <iostream>
#include <mutex>
#include <map>

#include <realm_types/frame.h>
#include <realm_types/structs.h>
#include <realm_types/camera_settings.h>
#include <realm_vslam/visual_slam_IF.h>
#include <realm_vslam/visual_slam_settings.h>


// CPU implementation
#include <ORB_SLAM2/Settings.h>
#include <ORB_SLAM2/System.h>
#include <ORB_SLAM2/KeyFrame.h>

namespace realm
{


class OrbSlam2 : public VisualSlamIF
{
  public:
    // Construction
    OrbSlam2(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set);
    ~OrbSlam2();

    // Process
    VisualSlamIF::State Track(Frame::Ptr &frame) override;
    void Close() override;
    void Reset() override;

    // Getter
    cv::Mat GetMapPoints() const override;
    cv::Mat GetTrackedMapPoints() const override;
    bool DrawTrackedImage(cv::Mat &) const override;

    // Transport from realm to ros
    //void SetKeyRosTransport(void (*func)(void *pKey)) override;
    void RegisterUpdateTransport(const VisualSlamIF::PoseUpdateFuncCb &func) override;
    void RegisterResetCallback(const VisualSlamIF::ResetFuncCb &func) override;

    void printSettingsToLog() override;

  private:

    int32_t _prev_keyid;
    double _resizing;

    bool _use_viewer;
    std::string _path_settings;
    std::string _path_vocabulary;
    ORB_SLAM2::CameraSettings _cam_set;
    ORB_SLAM2::TrackerSettings _track_set;
    ORB_SLAM2::ViewerSettings _view_set;
    cv::FileStorage _settings_file;


    // Map of orbslam inserted keyframe ids to realm frames
    std::map<uint32_t , uint32_t> _orb_to_frame_ids;

    // Cpu handles
    ORB_SLAM2::System *_slam;

    VisualSlamIF::PoseUpdateFuncCb _pose_update_func_cb;

    cv::Mat invertPose(const cv::Mat &pose) const;
    void keyframeUpdateCb(ORB_SLAM2::KeyFrame* kf);
};

}