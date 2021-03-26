
#ifndef OPENREALM_OV2_SLAM_H
#define OPENREALM_OV2_SLAM_H

#include <thread>

#include <realm_vslam_base/visual_slam_IF.h>
#include <realm_vslam_base/visual_slam_settings.h>
#include <realm_core/camera_settings.h>
#include <realm_core/timer.h>

#include <ov2slam/ov2slam.hpp>
#include <ov2slam/slam_params.hpp>

namespace realm
{

class Ov2Slam : public VisualSlamIF
{
public:
  Ov2Slam(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set);
  ~Ov2Slam();

  // Process
  VisualSlamIF::State track(Frame::Ptr &frame, const cv::Mat &T_c2w_initial) override;
  void close() override;
  void reset() override;

  PointCloud::Ptr getTrackedMapPoints() override;
  bool drawTrackedImage(cv::Mat &) const override;

  // Transport from realm to ros
  void registerUpdateTransport(const VisualSlamIF::PoseUpdateFuncCb &func) override {};
  void registerResetCallback(const VisualSlamIF::ResetFuncCb &func) override {};

  void printSettingsToLog() override;

private:

  double m_resizing;

  int m_id_previous;
  long m_t_first;

  uint32_t m_max_point_id;
  uint32_t m_base_point_id;

  std::unique_ptr<SlamManager> m_slam;
  std::shared_ptr<SlamParams>  m_slam_params;

  cv::Mat convertPose(const Eigen::Matrix<double, 3, 4> &mat_eigen);
};

}

#endif //OPENREALM_OV2_SLAM_H
