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

#include <realm_core/frame.h>
#include <realm_core/structs.h>
#include <realm_core/camera_settings.h>
#include <realm_vslam_base/visual_slam_IF.h>
#include <realm_vslam_base/visual_slam_settings.h>


// CPU implementation
#include <ORB_SLAM3/System.h>
#include <ORB_SLAM3/KeyFrame.h>

namespace realm
{


class OrbSlam : public VisualSlamIF
{
  public:
    // Construction
    OrbSlam(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set);
    ~OrbSlam();

    // Process
    VisualSlamIF::State track(Frame::Ptr &frame, const cv::Mat &T_c2w_initial) override;
    void close() override;
    void reset() override;

    // Getter
    cv::Mat getMapPoints() const override;
    cv::Mat getTrackedMapPoints() const override;
    bool drawTrackedImage(cv::Mat &) const override;

    // Transport from realm to ros
    //void SetKeyRosTransport(void (*func)(void *pKey)) override;
    void registerUpdateTransport(const VisualSlamIF::PoseUpdateFuncCb &func) override;
    void registerResetCallback(const VisualSlamIF::ResetFuncCb &func) override;

    void printSettingsToLog() override;

  private:

    int32_t m_prev_keyid;
    double m_resizing;

    uint64_t m_timestamp_reference;

    bool m_use_viewer;
    std::string m_path_settings;
    std::string m_path_vocabulary;
    cv::FileStorage m_settings_file;


    // Map of orbslam inserted keyframe ids to realm frames
    std::map<uint32_t , uint32_t> m_orb_to_frame_ids;

    // Cpu handles
    ORB_SLAM3::System *m_slam;

    VisualSlamIF::PoseUpdateFuncCb m_pose_update_func_cb;

    cv::Mat invertPose(const cv::Mat &pose) const;
    void keyframeUpdateCb(ORB_SLAM3::KeyFrame* kf);
};

}