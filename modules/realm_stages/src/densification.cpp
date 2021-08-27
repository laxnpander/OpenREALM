

#include <realm_stages/densification.h>

using namespace realm;
using namespace stages;

Densification::Densification(const StageSettings::Ptr &stage_set,
                             const DensifierSettings::Ptr &densifier_set,
                             double rate)
: StageBase("densification", (*stage_set)["path_output"].toString(), rate, (*stage_set)["queue_size"].toInt(), bool((*stage_set)["log_to_file"].toInt())),
  m_use_filter_bilat((*stage_set)["use_filter_bilat"].toInt() > 0),
  m_use_filter_guided((*stage_set)["use_filter_guided"].toInt() > 0),
  m_depth_min_current(0.0),
  m_depth_max_current(0.0),
  m_do_drop_planar((*stage_set)["compute_normals"].toInt() > 0),
  m_compute_normals((*stage_set)["compute_normals"].toInt() > 0),
  m_rcvd_frames(0),
  m_settings_save({(*stage_set)["save_bilat"].toInt() > 0,
                  (*stage_set)["save_dense"].toInt() > 0,
                  (*stage_set)["save_guided"].toInt() > 0,
                  (*stage_set)["save_imgs"].toInt() > 0,
                  (*stage_set)["save_sparse"].toInt() > 0,
                  (*stage_set)["save_thumb"].toInt() > 0,
                  (*stage_set)["save_normals"].toInt() > 0})
{
  registerAsyncDataReadyFunctor([=]{ return !m_buffer_reco.empty(); });

  m_densifier = densifier::DensifierFactory::create(densifier_set);
  m_n_frames = m_densifier->getNrofInputFrames();

  // Creation of reference plane, currently only the one below is supported
  m_plane_ref.pt = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
  m_plane_ref.n = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
}

void Densification::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateStatisticsIncoming();

  // Increment received valid frames
  m_rcvd_frames++;

  // Check if frame and settings are fulfilled to process/densify incoming frames
  // if not, redirect to next stage
  if (   !frame->isKeyframe()
      || !frame->hasAccuratePose()
      || !frame->isDepthComputed())
  {
    LOG_F(INFO, "Frame #%u:", frame->getFrameId());
    LOG_F(INFO, "Keyframe? %s", frame->isKeyframe() ? "Yes" : "No");
    LOG_F(INFO, "Accurate Pose? %s", frame->hasAccuratePose() ? "Yes" : "No");
    LOG_F(INFO, "Surface? %i Points", frame->getSparseCloud() != nullptr ? frame->getSparseCloud()->size() : 0);

    LOG_F(INFO, "Frame #%u not suited for dense reconstruction. Passing through...", frame->getFrameId());
    if (!m_do_drop_planar)
      m_transport_frame(frame, "output/frame");
    updateStatisticsBadFrame();
    return;
  }

  pushToBufferReco(frame);
  notify();
}

bool Densification::process()
{
  // NOTE: All depthmap maps are CV_32F except they are explicitly casted

  // First check if buffer has enough frames already, else don't do anything
  if (m_buffer_reco.size() < m_n_frames)
  {
    return false;
  }

  // Densification step using stereo
  long t = getCurrentTimeMilliseconds();
  Frame::Ptr frame_processed;
  Depthmap::Ptr depthmap = processStereoReconstruction(m_buffer_reco, frame_processed);
  popFromBufferReco();
  updateStatisticsProcessedFrame();

  LOG_IF_F(INFO, m_verbose, "Timing [Dense Reconstruction]: %lu ms", getCurrentTimeMilliseconds() - t);
  if (!depthmap)
    return true;

  // Compute normals if desired
  t = getCurrentTimeMilliseconds();
  cv::Mat normals;
  if (m_compute_normals)
    normals = stereo::computeNormalsFromDepthMap(depthmap->data());
  LOG_IF_F(INFO, m_verbose, "Timing [Computing Normals]: %lu ms", getCurrentTimeMilliseconds() - t);

  // Remove outliers
  double depth_min = frame_processed->getMedianSceneDepth()*0.25;
  double depth_max = frame_processed->getMedianSceneDepth()*1.75;
  depthmap = forceInRange(depthmap, depth_min, depth_max);
  LOG_F(INFO, "Scene depthmap forced in range %4.2f ... %4.2f", depth_min, depth_max);

  // Set data in the frame
  t = getCurrentTimeMilliseconds();
  frame_processed->setDepthmap(depthmap);
  LOG_IF_F(INFO, m_verbose, "Timing [Setting]: %lu ms", getCurrentTimeMilliseconds() - t);

  // Creating dense cloud
  cv::Mat img3d = stereo::reprojectDepthMap(depthmap->getCamera(), depthmap->data());
  cv::Mat dense_cloud = img3d.reshape(1, img3d.rows*img3d.cols);

  // Denoising
  t = getCurrentTimeMilliseconds();
  m_buffer_consistency.emplace_back(std::make_pair(frame_processed, dense_cloud));
  if (m_buffer_consistency.size() >= 4)
  {
    frame_processed = consistencyFilter(&m_buffer_consistency);
    m_buffer_consistency.pop_front();
  }
  else
  {
    LOG_IF_F(INFO, m_verbose, "Consistency filter is activated. Waiting for more frames for denoising...");
    return true;
  }
  LOG_IF_F(INFO, m_verbose, "Timing [Denoising]: %lu ms", getCurrentTimeMilliseconds() - t);

  // Last check if frame still has valid depthmap
  if (!frame_processed->getDepthmap())
  {
    //_transport_frame(frame_processed, "output/frame");
    return true;
  }

  // Post processing
  depthmap->data() = applyDepthMapPostProcessing(depthmap->data());

  // Savings
  t = getCurrentTimeMilliseconds();
  saveIter(frame_processed, normals);
  LOG_IF_F(INFO, m_verbose, "Timing [Saving]: %lu ms", getCurrentTimeMilliseconds() - t);

  // Republish frame to next stage
  t = getCurrentTimeMilliseconds();
  publish(frame_processed, depthmap->data());
  LOG_IF_F(INFO, m_verbose, "Timing [Publish]: %lu ms", getCurrentTimeMilliseconds() - t);

  return true;
}

Frame::Ptr Densification::consistencyFilter(std::deque<std::pair<Frame::Ptr, cv::Mat>>* buffer_denoise)
{
  Frame::Ptr frame = (*buffer_denoise)[buffer_denoise->size()/2].first;

  Depthmap::Ptr depthmap_ii = frame->getDepthmap();

  cv::Mat depthmap_ii_data = depthmap_ii->data();
  int rows = depthmap_ii_data.rows;
  int cols = depthmap_ii_data.cols;

  cv::Mat votes = cv::Mat::zeros(rows, cols, CV_8UC1);

  float th_depth = 0.1;

  for (const auto &f : *buffer_denoise)
  {
    if (f.first == frame)
      continue;

    cv::Mat dense_cloud = f.second;
    cv::Mat depthmap_ij_data = stereo::computeDepthMapFromPointCloud(depthmap_ii->getCamera(), dense_cloud);

    for (int r = 0; r < rows; ++r)
      for (int c =0; c < cols; ++c)
      {
        float d_ii = depthmap_ii_data.at<float>(r, c);
        float d_ij = depthmap_ij_data.at<float>(r, c);

        if (d_ii <= 0 || d_ij <= 0)
          continue;

        if (fabsf(d_ij-d_ii)/d_ii < th_depth)
          votes.at<uchar>(r, c) = votes.at<uchar>(r, c) + 1;
      }
  }

  cv::Mat mask = (votes < 2);
  depthmap_ii_data.setTo(-1.0, mask);

  double perc_coverage = 100 - (static_cast<double>(cv::countNonZero(mask)) / (mask.rows*mask.cols) * 100.0);

  if (perc_coverage < 30.0)
  {
    LOG_IF_F(WARNING, m_verbose, "Depthmap coverage too low (%3.1f%%). Assuming plane surface.", perc_coverage);
    frame->setDepthmap(nullptr);

    for (auto it = buffer_denoise->begin(); it != buffer_denoise->end(); ) {
      if (it->first->getFrameId() == frame->getFrameId())
      {
        it = buffer_denoise->erase(it);
        break;
      }
      else
        ++it;
    }
  }
  else
  {
    LOG_IF_F(INFO, m_verbose, "Depthmap coverage left after denoising: %3.1f%%", perc_coverage);
  }

  return frame;
}

Depthmap::Ptr Densification::processStereoReconstruction(const std::deque<Frame::Ptr> &buffer, Frame::Ptr &frame_processed)
{
  LOG_F(INFO, "Performing stereo reconstruction...");

  // Reference frame is the one in the middle (if more than two)
  int ref_idx = (int)buffer.size()/2;
  frame_processed = buffer[ref_idx];

  // Compute baseline information for all frames
  std::vector<double> baselines;
  baselines.reserve(buffer.size());

  std::string stringbuffer;
  for (auto &f : buffer)
  {
    if (f == frame_processed)
      continue;

    baselines.push_back(stereo::computeBaselineFromPose(frame_processed->getPose(), f->getPose()));
    stringbuffer += std::to_string(baselines.back()) + "m ";
  }
  LOG_F(INFO, "Baselines to reference frame: %s", stringbuffer.c_str());

  LOG_F(INFO, "Reconstructing frame #%u...", frame_processed->getFrameId());
  Depthmap::Ptr depthmap = m_densifier->densify(buffer, (uint8_t)ref_idx);

  LOG_IF_F(INFO, depthmap != nullptr, "Successfully reconstructed frame!");
  LOG_IF_F(WARNING, depthmap == nullptr, "Reconstruction failed!");

  return depthmap;
}

Depthmap::Ptr Densification::forceInRange(const Depthmap::Ptr &depthmap, double min_depth, double max_depth)
{
  cv::Mat mask;
  cv::Mat data = depthmap->data();

  cv::inRange(data, min_depth, max_depth, mask);

  cv::bitwise_not(mask, mask);
  data.setTo(-1.0f, mask);

  return depthmap;
}

cv::Mat Densification::applyDepthMapPostProcessing(const cv::Mat &depthmap)
{
  cv::Mat depthmap_filtered;
  if (m_use_filter_bilat)
      cv::bilateralFilter(depthmap, depthmap_filtered, 5, 25, 25);

  /*if (_settings_save.save_bilat)
    io::saveImageColorMap(depthmap_filtered, _depth_min_current, _depth_max_current, _stage_path + "/bilat", "bilat",
                          _frame_current->getFrameId(), io::ColormapType::DEPTH);*/

  return depthmap_filtered;
}

cv::Mat Densification::computeDepthMapMask(const cv::Mat &depth_map, bool use_sparse_mask)
{
  cv::Mat mask1, mask2, mask3;

  /*if (use_sparse_mask)
    densifier::computeSparseMask(_frame_current->getSparseCloud(), _frame_current->getResizedCamera(), mask1);
  else
    mask1 = cv::Mat::ones(depth_map.rows, depth_map.cols, CV_8UC1)*255;

  cv::inRange(depth_map, _depth_min_current, _depth_max_current, mask2);
  cv::bitwise_and(mask1, mask2, mask3);*/
  return mask3;
}

void Densification::publish(const Frame::Ptr &frame, const cv::Mat &depthmap)
{
  // First update statistics about outgoing frame rate
  updateStatisticsOutgoing();

  m_transport_frame(frame, "output/frame");
  m_transport_pose(frame->getPose(), frame->getGnssUtm().zone, frame->getGnssUtm().band, "output/pose");
  m_transport_img(frame->getResizedImageUndistorted(), "output/img_rectified");
  m_transport_depth_map(depthmap, "output/depth");
  m_transport_pointcloud(frame->getSparseCloud(), "output/pointcloud");

  cv::Mat depthmap_display;
  cv::normalize(depthmap, depthmap_display, 0, 65535, cv::NormTypes::NORM_MINMAX, CV_16UC1, (depthmap > 0));
  m_transport_img(depthmap_display, "output/depth_display");
}

void Densification::saveIter(const Frame::Ptr &frame, const cv::Mat &normals)
{
  cv::Mat depthmap_data = frame->getDepthmap()->data();

  if (m_settings_save.save_imgs)
    io::saveImage(frame->getResizedImageUndistorted(), io::createFilename(m_stage_path + "/imgs/imgs_", frame->getFrameId(), ".png"));
  if (m_settings_save.save_normals && m_compute_normals && !normals.empty())
    io::saveImageColorMap(normals, (depthmap_data > 0), m_stage_path + "/normals", "normals", frame->getFrameId(), io::ColormapType::NORMALS);
  if (m_settings_save.save_sparse)
  {
    //cv::Mat depthmap_sparse = stereo::computeDepthMapFromPointCloud(frame->getResizedCamera(), frame->getSparseCloud()->data().colRange(0, 3));
    //io::saveDepthMap(depthmap_sparse, m_stage_path + "/sparse/sparse_%06i.tif", frame->getFrameId());
  }
  if (m_settings_save.save_dense)
    io::saveDepthMap(frame->getDepthmap(), m_stage_path + "/dense/dense_%06i.tif", frame->getFrameId());
}

void Densification::pushToBufferReco(const Frame::Ptr &frame)
{
  std::unique_lock<std::mutex> lock(m_mutex_buffer_reco);

  m_buffer_reco.push_back(frame);

  if (m_buffer_reco.size() > m_queue_size)
  {
    m_buffer_reco.pop_front();
    updateStatisticsSkippedFrame();
  }
}

void Densification::popFromBufferReco()
{
  std::unique_lock<std::mutex> lock(m_mutex_buffer_reco);
  m_buffer_reco.pop_front();
}

void Densification::reset()
{
  // TODO: Reset in _densifier
  m_rcvd_frames = 0;
  LOG_F(INFO, "Densification Stage: RESETED!");
}

void Densification::initStageCallback()
{
  // If we aren't saving any information, skip directory creation
  if (!(m_log_to_file || m_settings_save.save_required()))
  {
    return;
  }

  // Stage directory first
  if (!io::dirExists(m_stage_path))
    io::createDir(m_stage_path);

  // Then sub directories
  if (!io::dirExists(m_stage_path + "/sparse") && m_settings_save.save_sparse)
    io::createDir(m_stage_path + "/sparse");
  if (!io::dirExists(m_stage_path + "/dense") && m_settings_save.save_dense)
    io::createDir(m_stage_path + "/dense");
  if (!io::dirExists(m_stage_path + "/bilat") && m_settings_save.save_bilat)
    io::createDir(m_stage_path + "/bilat");
  if (!io::dirExists(m_stage_path + "/guided") && m_settings_save.save_guided)
    io::createDir(m_stage_path + "/guided");
  if (!io::dirExists(m_stage_path + "/normals") && m_settings_save.save_normals)
    io::createDir(m_stage_path + "/normals");
  if (!io::dirExists(m_stage_path + "/imgs") && m_settings_save.save_imgs)
    io::createDir(m_stage_path + "/imgs");
  if (!io::dirExists(m_stage_path + "/thumb") && m_settings_save.save_thumb)
    io::createDir(m_stage_path + "/thumb");
}

void Densification::printSettingsToLog()
{
  LOG_F(INFO, "### Stage process settings ###");
  LOG_F(INFO, "- use_filter_bilat: %i", m_use_filter_bilat);
  LOG_F(INFO, "- use_filter_guided: %i", m_use_filter_guided);
  LOG_F(INFO, "- compute_normals: %i", m_compute_normals);

  LOG_F(INFO, "### Stage save settings ###");
  LOG_F(INFO, "- save_bilat: %i", m_settings_save.save_bilat);
  LOG_F(INFO, "- save_dense: %i", m_settings_save.save_dense);
  LOG_F(INFO, "- save_guided: %i", m_settings_save.save_guided);
  LOG_F(INFO, "- save_imgs: %i", m_settings_save.save_imgs);
  LOG_F(INFO, "- save_normals: %i", m_settings_save.save_normals);
  LOG_F(INFO, "- save_sparse: %i", m_settings_save.save_sparse);
  LOG_F(INFO, "- save_thumb: %i", m_settings_save.save_thumb);

  m_densifier->printSettingsToLog();
}

uint32_t Densification::getQueueDepth() {
  return m_buffer_reco.size();
}
