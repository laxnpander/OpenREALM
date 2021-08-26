

#pragma once

#include <iostream>
#include <mutex>
#include <map>

#include <realm_core/frame.h>
#include <realm_core/structs.h>
#include <realm_core/imu_settings.h>
#include <realm_core/camera_settings.h>
#include <realm_vslam_base/visual_slam_IF.h>
#include <realm_vslam_base/visual_slam_settings.h>

#ifdef USE_ORB_SLAM2
#include <ORB_SLAM2/System.h>
#include <ORB_SLAM2/KeyFrame.h>
namespace ORB_SLAM = ORB_SLAM2;
#endif

#ifdef USE_ORB_SLAM3
#include <ORB_SLAM3/System.h>
#include <ORB_SLAM3/KeyFrame.h>
namespace ORB_SLAM = ORB_SLAM3;
#endif

namespace realm
{


class OrbSlam : public VisualSlamIF
{
  public:
    // Construction
    OrbSlam(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set, const ImuSettings::Ptr &imu_set = nullptr);
    ~OrbSlam();

    // Process
    VisualSlamIF::State track(Frame::Ptr &frame, const cv::Mat &T_c2w_initial) override;
    void close() override;
    void reset() override;

    // Getter
    PointCloud::Ptr getTrackedMapPoints() override;
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
    ORB_SLAM::System *m_slam;

    VisualSlamIF::PoseUpdateFuncCb m_pose_update_func_cb;

    cv::Mat invertPose(const cv::Mat &pose) const;
    void keyframeUpdateCb(ORB_SLAM::KeyFrame* kf);

    // ORB_SLAM3 only
#ifdef USE_ORB_SLAM3
  std::vector<ORB_SLAM::IMU::Point> m_imu_queue;

  void queueImuData(const ImuData &data) override;
#endif
};

}