

#include <realm_vslam_base/open_vslam.h>
#include <realm_core/timer.h>

#include <openvslam/config.h>
#include <openvslam/data/landmark.h>
#include <openvslam/publish/map_publisher.h>
#include <openvslam/publish/frame_publisher.h>
#include <future>

using namespace realm;

OpenVslam::OpenVslam(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set)
 : m_max_point_id(0),
   m_base_point_id(0),
   m_nrof_keyframes(0),
   m_previous_state(openvslam::tracker_state_t::NotInitialized),
   m_last_keyframe(nullptr),
   m_max_keyframe_links(10),
   m_resizing((*vslam_set)["resizing"].toDouble()),
   m_path_vocabulary((*vslam_set)["path_vocabulary"].toString())
{
  // ov: OpenVSLAM
  // or: OpenREALM
  YAML::Node yaml_config;

  YAML::Node yaml_cam;
  yaml_cam["name"]  = "cam";
  yaml_cam["model"] = "perspective";
  yaml_cam["setup"] = "monocular";
  yaml_cam["color_order"] = "RGB";
  yaml_cam["fx"] = (*cam_set)["fx"].toDouble() * m_resizing;
  yaml_cam["fy"] = (*cam_set)["fy"].toDouble() * m_resizing;
  yaml_cam["cx"] = (*cam_set)["cx"].toDouble() * m_resizing;
  yaml_cam["cy"] = (*cam_set)["cy"].toDouble() * m_resizing;
  yaml_cam["k1"] = (*cam_set)["k1"].toDouble();
  yaml_cam["k2"] = (*cam_set)["k2"].toDouble();
  yaml_cam["p1"] = (*cam_set)["p1"].toDouble();
  yaml_cam["p2"] = (*cam_set)["p2"].toDouble();
  yaml_cam["k3"] = (*cam_set)["k3"].toDouble();
  yaml_cam["fps"] = (*cam_set)["fps"].toDouble();
  yaml_cam["cols"] = static_cast<unsigned int>((*cam_set)["width"].toInt() * m_resizing);
  yaml_cam["rows"] = static_cast<unsigned int>((*cam_set)["height"].toInt() * m_resizing);
  yaml_config["Camera"] = yaml_cam;

  YAML::Node yaml_features;
  yaml_features["max_num_keypoints"] = (*vslam_set)["nrof_features"].toInt();
  yaml_features["ini_max_num_keypoints"] = 2*(*vslam_set)["nrof_features"].toInt();
  yaml_features["scale_factor"] = (*vslam_set)["scale_factor"].toFloat();
  yaml_features["num_levels"] = (*vslam_set)["n_pyr_levels"].toInt();;
  yaml_features["ini_fast_threshold"] = (*vslam_set)["ini_th_FAST"].toInt();
  yaml_features["min_fast_threshold"] = (*vslam_set)["min_th_FAST"].toInt();
  yaml_config["Feature"] = yaml_features;

  YAML::Node yaml_mapping;
  yaml_mapping["baseline_dist_thr_ratio"] = 0.02;
  yaml_config["Mapping"] = yaml_mapping;

  m_config = std::make_shared<openvslam::config>(yaml_config, "");
  m_vslam = std::make_shared<openvslam::system>(m_config, m_path_vocabulary);
  m_frame_publisher = m_vslam->get_frame_publisher();
  m_map_publisher = m_vslam->get_map_publisher();

  m_vslam->startup();
}

VisualSlamIF::State OpenVslam::track(Frame::Ptr &frame, const cv::Mat &T_c2w_initial)
{
  // Set image resizing accoring to settings
  frame->setImageResizeFactor(m_resizing);

  // OpenVSLAM returns a transformation from the world to the camera frame (T_w2c). In case we provide an initial guess
  // of the current pose, we have to invert this before, because in OpenREALM the standard is defined as T_c2w.
  cv::Mat T_w2c;
  std::shared_ptr<openvslam::Mat44_t> T_w2c_eigen;
  if (T_c2w_initial.empty())
  {
    T_w2c_eigen = m_vslam->feed_monocular_frame(frame->getResizedImageRaw(), frame->getTimestamp() * 10e-9);

    if (T_w2c_eigen != nullptr)
      T_w2c = convertToCv(*T_w2c_eigen);
  }
  else
  {
    // prior not yet implemented
  }

  openvslam::tracker_state_t tracker_state = m_vslam->get_tracker_state();

  // Draw frame of tracked features
  m_mutex_last_drawn_frame.lock();
  m_last_drawn_frame = m_frame_publisher->draw_frame();
  m_mutex_last_drawn_frame.unlock();

  // In case tracking was successful and slam not lost
  if (tracker_state == openvslam::tracker_state_t::Tracking)
  {
    // Get list of keyframes
    std::vector<openvslam::data::keyframe*> keyframes;
    unsigned int current_nrof_keyframes = m_map_publisher->get_keyframes(keyframes);

    // Not ideal implementation, but I am not sure that the keyframes are sorted
    m_mutex_last_keyframe.lock();
    if (m_last_keyframe == nullptr)
      m_last_keyframe = keyframes.back();
    else
    {
      for (auto kf : keyframes)
        if (kf->id_ > m_last_keyframe->id_)
          m_last_keyframe = kf;
    }
    m_mutex_last_keyframe.unlock();

    // Pose definition as 3x4 matrix, calculated as 4x4 with last row (0, 0, 0, 1)
    // OpenVSLAM pose is defined as T_w2c, however the more intuitive way to describe
    // it for mapping is T_c2w (camera to world) therefore invert the pose matrix
    cv::Mat T_c2w = invertPose(T_w2c);

    // Remove last row of 0,0,0,1
    T_c2w.pop_back();
    frame->setVisualPose(T_c2w);

    PointCloud::Ptr sparse_cloud = getTrackedMapPoints();
    frame->setSparseCloud(sparse_cloud, true);

    m_previous_state = tracker_state;

    // Check current state of the slam
    if (m_nrof_keyframes == 0 && current_nrof_keyframes > 0)
    {
      m_nrof_keyframes = current_nrof_keyframes;
      return State::INITIALIZED;
    }
    else if (current_nrof_keyframes != m_nrof_keyframes)
    {
      // We want to keep all keyframes once created
      m_last_keyframe->set_not_to_be_erased();

      // Add to keyframe links for future updates
      addKeyframeLink(frame, m_last_keyframe);

      // Make sure our future is either valid or ready, meaning computation is complete
      if (!m_future_update_keyframes.valid() || m_future_update_keyframes.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        m_future_update_keyframes = std::async(std::launch::async, [=](){ updateKeyframes(); });
      } else {
        LOG_F(WARNING, "OpenVSLAM updateKeyframes did not finish before next frame was processed. Skipping update!");
      }
      m_nrof_keyframes = current_nrof_keyframes;
      return State::KEYFRAME_INSERT;
    }
    else
    {
      return State::FRAME_INSERT;
    }
  }
  else if ((m_previous_state == openvslam::tracker_state_t::Tracking || m_previous_state == openvslam::tracker_state_t::Lost) &&
            (tracker_state == openvslam::tracker_state_t::NotInitialized || tracker_state == openvslam::tracker_state_t::Initializing)) {
    // If we had tracking, then lost it then OpenVSlam initiated a reset and we should reset our local frames as well
    LOG_F(INFO, "Internal OpenVSLAM reset detected (State: %i was %i)", (int)tracker_state, (int)m_previous_state);
    internalReset();
    m_reset_callback();
  }

  m_previous_state = tracker_state;

  return State::LOST;
}

void OpenVslam::close()
{
  // In the current OpenVSLAM build, we don't have a way to know we shut
  // down other than seeing if we requested a terminate previously.  The internal flag isn't exposed.
  // So, we assume if we've requested termination, that we have also shut down.
  // This prevents a crash that could occur if close() was accidentally called twice.
  if (!m_vslam->terminate_is_requested())
  {
    m_vslam->request_terminate();
    m_vslam->shutdown();
  }
}

void OpenVslam::reset()
{
  LOG_F(INFO, "Reseting visual SLAM...");
  m_vslam->request_reset();
}

void OpenVslam::internalReset()
{
  std::lock_guard<std::mutex> lock(m_mutex_last_keyframe);
  m_last_keyframe = nullptr;
  m_nrof_keyframes = 0;

  // The new base point id is the maximum point id ever recognised +1. This way even though the SLAM starts counting
  // at 0 again, we still have a unique id for all the points.
  m_base_point_id = m_max_point_id+1;

  m_keyframe_links.clear();
}

void OpenVslam::registerResetCallback(const VisualSlamIF::ResetFuncCb &func)
{
  m_reset_callback = func;
}

PointCloud::Ptr OpenVslam::getTrackedMapPoints()
{
  m_mutex_last_keyframe.lock();
  std::vector<openvslam::data::landmark*> landmarks = m_last_keyframe->get_landmarks();
  m_mutex_last_keyframe.unlock();

  std::vector<uint32_t> point_ids;
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

    uint32_t point_id = extractPointId(lm);
    point_ids.push_back(point_id);

    // Save the maximum extracted point id
    if (point_id > m_max_point_id)
      m_max_point_id = point_id;
  }

  return std::make_shared<PointCloud>(point_ids, points);
}

bool OpenVslam::drawTrackedImage(cv::Mat &img) const
{
  img = getLastDrawnFrame();
  return !img.empty();
}

uint32_t OpenVslam::extractPointId(openvslam::data::landmark* lm)
{
  return m_base_point_id + lm->id_;
}

cv::Mat OpenVslam::getLastDrawnFrame() const
{
  std::lock_guard<std::mutex> lock(m_mutex_last_drawn_frame);
  return m_last_drawn_frame.clone();
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

  // Invert pose
  cv::Mat mat_cv_inv = cv::Mat::eye(4, 4, mat_cv.type());
  cv::Mat R_t = (mat_cv.rowRange(0, 3).colRange(0, 3)).t();
  cv::Mat t = -R_t*mat_cv.rowRange(0, 3).col(3);
  R_t.copyTo(mat_cv_inv.rowRange(0, 3).colRange(0, 3));
  t.copyTo(mat_cv_inv.rowRange(0, 3).col(3));

  return mat_cv_inv;
}

void OpenVslam::printSettingsToLog()
{

}

void OpenVslam::queueImuData(const VisualSlamIF::ImuData &imu)
{
  // TBD
}

void OpenVslam::addKeyframeLink(Frame::Ptr &frame_realm, openvslam::data::keyframe* frame_ovslam)
{
  // Keep only a maximum number of links, so the list does not grow indefinitely
  if (m_keyframe_links.size() > m_max_keyframe_links)
    m_keyframe_links.pop_front();

  m_keyframe_links.emplace_back(std::make_pair(frame_realm, m_last_keyframe));
}

void OpenVslam::updateKeyframes()
{
  long t = Timer::getCurrentTimeMilliseconds();

  for (auto it = m_keyframe_links.begin(); it != m_keyframe_links.end(); it++)
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
      PointCloud::Ptr sparse_cloud = frame_realm->getSparseCloud();
      std::vector<openvslam::data::landmark*> landmarks = frame_slam->get_landmarks();

      cv::Mat new_surface_points;
      new_surface_points.reserve(landmarks.size());

      std::vector<uint32_t> new_surface_point_ids;
      for (const auto &lm : landmarks)
      {
        if (!lm || lm->will_be_erased())
        {
          continue;
        }
        openvslam::Vec3_t pos = lm->get_pos_in_world();
        cv::Mat pt = (cv::Mat_<double>(1, 3) << pos[0], pos[1], pos[2]);
        new_surface_points.push_back(pt);
        new_surface_point_ids.push_back(lm->id_);
      }

      if (sparse_cloud->size() != new_surface_points.rows)
      {
        LOG_F(INFO, "Updating frame %u: %u --> %u", frame_realm->getFrameId(), sparse_cloud->size(), new_surface_points.rows);
        frame_realm->setSparseCloud(std::make_shared<PointCloud>(new_surface_point_ids, new_surface_points), true);
      }
    }
    else
    {
      LOG_F(INFO, "Frame object out of scope. Deleting reference.");
      // Frame is not existing anymore, delete from dequeue
      it = m_keyframe_links.erase(it);
    }
  }
  LOG_F(INFO, "Timing [Update KFs]: %lu ms", Timer::getCurrentTimeMilliseconds()-t);
}