

#include <realm_vslam_base/open_vslam.h>

#include <openvslam/config.h>
#include <openvslam/data/landmark.h>
#include <openvslam/publish/map_publisher.h>
#include <openvslam/publish/frame_publisher.h>

using namespace realm;

OpenVslam::OpenVslam(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set)
 : m_nrof_keyframes(0),
   m_last_keyframe(nullptr),
   m_resizing((*vslam_set)["resizing"].toDouble()),
   m_path_vocabulary((*vslam_set)["path_vocabulary"].toString()),
   m_keyframe_updater(new OpenVslamKeyframeUpdater("Keyframe Updater", 100, false))
{
  // ov: OpenVSLAM
  // or: OpenREALM
  YAML::Node settings;
  settings["Camera.name"] = "cam";
  settings["Camera.setup"] = "monocular";
  settings["Camera.model"] = "perspective";
  settings["Camera.color_order"] = "RGB";
  settings["Camera.fx"] = (*cam_set)["fx"].toDouble() * m_resizing;
  settings["Camera.fy"] = (*cam_set)["fy"].toDouble() * m_resizing;
  settings["Camera.cx"] = (*cam_set)["cx"].toDouble() * m_resizing;
  settings["Camera.cy"] = (*cam_set)["cy"].toDouble() * m_resizing;
  settings["Camera.k1"] = (*cam_set)["k1"].toDouble();
  settings["Camera.k2"] = (*cam_set)["k2"].toDouble();
  settings["Camera.p1"] = (*cam_set)["p1"].toDouble();
  settings["Camera.p2"] = (*cam_set)["p2"].toDouble();
  settings["Camera.k3"] = (*cam_set)["k3"].toDouble();
  settings["Camera.fps"] = (*cam_set)["fps"].toDouble();
  settings["Camera.cols"] = static_cast<unsigned int>((*cam_set)["width"].toInt() * m_resizing);
  settings["Camera.rows"] = static_cast<unsigned int>((*cam_set)["height"].toInt() * m_resizing);
  settings["Feature.max_num_keypoints"] = (*vslam_set)["nrof_features"].toInt();
  settings["Feature.scale_factor"] = (*vslam_set)["scale_factor"].toFloat();
  settings["Feature.ini_fast_threshold"] = (*vslam_set)["ini_th_FAST"].toInt();
  settings["Feature.min_fast_threshold"] = (*vslam_set)["min_th_FAST"].toInt();

  m_config = std::make_shared<openvslam::config>(settings, "");
  m_vslam = std::make_shared<openvslam::system>(m_config, m_path_vocabulary);
  m_frame_publisher = m_vslam->get_frame_publisher();
  m_map_publisher = m_vslam->get_map_publisher();

  m_vslam->startup();
  m_keyframe_updater->start();
}

VisualSlamIF::State OpenVslam::track(Frame::Ptr &frame, const cv::Mat &T_c2w_initial)
{
  // Set image resizing accoring to settings
  frame->setImageResizeFactor(m_resizing);

  // ORB SLAM returns a transformation from the world to the camera frame (T_w2c). In case we provide an initial guess
  // of the current pose, we have to invert this before, because in OpenREALM the standard is defined as T_c2w.
  cv::Mat T_w2c;
  openvslam::Mat44_t T_w2c_eigen;
  if (T_c2w_initial.empty())
  {
    T_w2c_eigen = m_vslam->feed_monocular_frame(frame->getResizedImageRaw(), frame->getTimestamp() * 10e-9);
    T_w2c = convertToCv(T_w2c_eigen);
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

  // In case tracking was successfull and slam not lost
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

    cv::Mat surface_pts = getTrackedMapPoints();
    frame->setSparseCloud(surface_pts, true);

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

      // Pass to keyframe updater, which regularly checks if points or pose have changed
      m_keyframe_updater->add(frame, m_last_keyframe);

      m_nrof_keyframes = current_nrof_keyframes;
      return State::KEYFRAME_INSERT;
    }
    else
    {
      return State::FRAME_INSERT;
    }
  }

  return State::LOST;
}

void OpenVslam::close()
{
  m_vslam->request_terminate();
  m_vslam->shutdown();
  m_keyframe_updater->requestFinish();
  m_keyframe_updater->join();
}

void OpenVslam::reset()
{
  LOG_F(INFO, "Reseting visual SLAM...");
  m_vslam->request_reset();
  m_keyframe_updater->requestReset();

  std::lock_guard<std::mutex> lock(m_mutex_last_keyframe);
  m_last_keyframe = nullptr;
  m_nrof_keyframes = 0;
  LOG_F(INFO, "Finished reseting visual SLAM.");
}

cv::Mat OpenVslam::getTrackedMapPoints() const
{
  m_mutex_last_keyframe.lock();
  std::vector<openvslam::data::landmark*> landmarks = m_last_keyframe->get_landmarks();
  m_mutex_last_keyframe.unlock();

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
  }
  return points;
}

bool OpenVslam::drawTrackedImage(cv::Mat &img) const
{
  img = getLastDrawnFrame();
  return !img.empty();
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
  return mat_cv;
}

void OpenVslam::printSettingsToLog()
{

}

void OpenVslam::queueImuData(const VisualSlamIF::ImuData &imu)
{
  // TBD
}

OpenVslamKeyframeUpdater::OpenVslamKeyframeUpdater(const std::string &thread_name, int64_t sleep_time, bool verbose)
  : WorkerThreadBase(thread_name, sleep_time, verbose)
{
}

void OpenVslamKeyframeUpdater::add(const std::weak_ptr<Frame> &frame_realm, openvslam::data::keyframe *frame_vslam)
{
  // We create a connection between the OpenREALM frames and the OpenVSLAM keyframes, so we can update points and
  // poses easily
  m_keyframe_links.emplace_back(std::make_pair(frame_realm, frame_vslam));
}

bool OpenVslamKeyframeUpdater::process()
{
  bool has_processed = false;

  for (auto it = m_keyframe_links.begin(); it != m_keyframe_links.end(); )
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
      cv::Mat surface_points = frame_realm->getSparseCloud();
      std::vector<openvslam::data::landmark*> landmarks = frame_slam->get_landmarks();

      cv::Mat new_surface_points;
      new_surface_points.reserve(landmarks.size());

      for (const auto &lm : landmarks)
      {
        if (!lm || lm->will_be_erased())
        {
          continue;
        }
        openvslam::Vec3_t pos = lm->get_pos_in_world();
        cv::Mat pt = (cv::Mat_<double>(1, 3) << pos[0], pos[1], pos[2]);
        new_surface_points.push_back(pt);
      }

      if (surface_points.rows != new_surface_points.rows)
      {
        LOG_IF_F(INFO, m_verbose, "Updating frame %u: %u --> %u", frame_realm->getFrameId(), surface_points.rows, new_surface_points.rows);
        frame_realm->setSparseCloud(new_surface_points, true);
      }

      has_processed = true;
    }
    else
    {
      LOG_IF_F(INFO, m_verbose, "Frame object out of scope. Deleting reference.");
      // Frame is not existing anymore, delete from dequeue
      it = m_keyframe_links.erase(it);
    }
  }
  return has_processed;
}

void OpenVslamKeyframeUpdater::reset()
{
  m_keyframe_links.clear();
}
