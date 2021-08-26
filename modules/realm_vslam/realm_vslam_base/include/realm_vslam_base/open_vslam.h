

#ifndef OPENREALM_OPEN_VSLAM_H
#define OPENREALM_OPEN_VSLAM_H

#include <realm_vslam_base/visual_slam_IF.h>
#include <realm_vslam_base/visual_slam_settings.h>
#include <realm_core/camera_settings.h>
#include <realm_core/worker_thread_base.h>

#include <openvslam/system.h>
#include <openvslam/type.h>
#include <openvslam/data/keyframe.h>
#include <openvslam/data/landmark.h>
#include <yaml-cpp/yaml.h>
#include <future>

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

  void queueImuData(const VisualSlamIF::ImuData &imu) override;

  bool drawTrackedImage(cv::Mat &img) const override;

  PointCloud::Ptr getTrackedMapPoints() override;

private:

  //! We need to assign points a unique id, so they can be identified across several resets of the visual SLAM system.
  //! We do that by saving the maximum, recognized point id and using this as starting point on a reset. The m_base_point_id
  //! is set to the max id on reset.
  uint32_t m_max_point_id;
  uint32_t m_base_point_id;

  unsigned int m_nrof_keyframes;

  double m_resizing;

  std::string m_path_vocabulary;

  mutable std::mutex m_mutex_last_drawn_frame;
  cv::Mat m_last_drawn_frame;

  mutable std::mutex m_mutex_last_keyframe;
  openvslam::data::keyframe* m_last_keyframe;

  unsigned int m_max_keyframe_links;
  std::list<std::pair<std::weak_ptr<Frame>, openvslam::data::keyframe*>> m_keyframe_links;

  openvslam::tracker_state_t m_previous_state;

  std::shared_ptr<openvslam::system> m_vslam;
  std::shared_ptr<openvslam::config> m_config;
  std::shared_ptr<openvslam::publish::frame_publisher> m_frame_publisher;
  std::shared_ptr<openvslam::publish::map_publisher> m_map_publisher;

  VisualSlamIF::ResetFuncCb m_reset_callback;

  std::future<void> m_future_update_keyframes;

  uint32_t extractPointId(openvslam::data::landmark* lm);

  cv::Mat getLastDrawnFrame() const;
  cv::Mat invertPose(const cv::Mat &pose) const;
  cv::Mat convertToCv(const openvslam::Mat44_t &mat) const;

  void internalReset();
  void addKeyframeLink(Frame::Ptr &frame_realm, openvslam::data::keyframe* frame_ovslam);
  void updateKeyframes();
  void registerResetCallback(const ResetFuncCb &func) override;
};

} // namespace realm

#endif //OPENREALM_OPEN_VSLAM_H

