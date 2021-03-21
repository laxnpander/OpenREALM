

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

  void queueImuData(const VisualSlamIF::ImuData &imu) override;

  bool drawTrackedImage(cv::Mat &img) const override;

  SparseCloud::Ptr getTrackedMapPoints() const override;

private:

  unsigned int m_nrof_keyframes;

  double m_resizing;

  std::string m_path_vocabulary;

  mutable std::mutex m_mutex_last_drawn_frame;
  cv::Mat m_last_drawn_frame;

  mutable std::mutex m_mutex_last_keyframe;
  openvslam::data::keyframe* m_last_keyframe;

  std::shared_ptr<openvslam::system> m_vslam;
  std::shared_ptr<openvslam::config> m_config;
  std::shared_ptr<openvslam::publish::frame_publisher> m_frame_publisher;
  std::shared_ptr<openvslam::publish::map_publisher> m_map_publisher;
  std::unique_ptr<OpenVslamKeyframeUpdater> m_keyframe_updater;

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
  std::list<std::pair<std::weak_ptr<Frame>, openvslam::data::keyframe*>> m_keyframe_links;

  bool process() override;

  void reset() override;

};

} // namespace realm

#endif //OPENREALM_OPEN_VSLAM_H

