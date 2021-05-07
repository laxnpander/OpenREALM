#include <realm_vslam_base/ov2_slam.h>
#include <realm_core/timer.h>

using namespace realm;

Ov2Slam::Ov2Slam(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set)
  : m_resizing(1.0),
    m_id_previous(-1),
    m_t_first(0),
    m_max_point_id(0),
    m_base_point_id(0)
{
  m_resizing = (*vslam_set)["resizing"].toDouble();

  m_slam_params = std::make_shared<SlamParams>();
  m_slam_params->blocalba_is_on_              = false;
  m_slam_params->blc_is_on_                   = false;
  m_slam_params->bvision_init_                = false;
  m_slam_params->breset_req_                  = false;
  m_slam_params->debug_                       = false;
  m_slam_params->log_timings_                 = false;
  m_slam_params->mono_                        = true;
  m_slam_params->stereo_                      = false;
  m_slam_params->bforce_realtime_             = (*vslam_set)["force_realtime"].toInt() > 0;
  m_slam_params->slam_mode_                   = true;
  m_slam_params->buse_loop_closer_            = (*vslam_set)["buse_loop_closer"].toInt() > 0;
  m_slam_params->bdo_stereo_rect_             = false;
  m_slam_params->alpha_                       = (*vslam_set)["alpha"].toDouble();
  m_slam_params->finit_parallax_              = (*vslam_set)["finit_parallax"].toFloat();
  m_slam_params->use_shi_tomasi_              = (*vslam_set)["use_shi_tomasi"].toInt() > 0;
  m_slam_params->use_brief_                   = (*vslam_set)["use_brief"].toInt() > 0;
  m_slam_params->use_fast_                    = (*vslam_set)["use_fast"].toInt() > 0;
  m_slam_params->use_singlescale_detector_    = (*vslam_set)["use_singlescale_detector"].toInt() > 0;
  m_slam_params->nmaxdist_                    = (*vslam_set)["nmax_dist"].toInt();
  m_slam_params->nfast_th_                    = (*vslam_set)["nfast_th"].toInt();
  m_slam_params->dmaxquality_                 = (*vslam_set)["dmax_quality"].toDouble();
  m_slam_params->use_clahe_                   = (*vslam_set)["use_clahe"].toInt() > 0;
  m_slam_params->fclahe_val_                  = (*vslam_set)["fclahe_val"].toFloat();
  m_slam_params->do_klt_                      = (*vslam_set)["do_klt"].toInt() > 0;
  m_slam_params->klt_use_prior_               = (*vslam_set)["klt_use_prior"].toInt() > 0;
  m_slam_params->btrack_keyframetoframe_      = (*vslam_set)["btrack_keyframetoframe"].toInt() > 0;
  m_slam_params->nklt_win_size_               = (*vslam_set)["nklt_win_size"].toInt();
  m_slam_params->klt_win_size_                = cv::Size2i(m_slam_params->nklt_win_size_, m_slam_params->nklt_win_size_);
  m_slam_params->nklt_pyr_lvl_                = (*vslam_set)["nklt_pyr_lvl"].toInt();
  m_slam_params->nmax_iter_                   = (*vslam_set)["nmax_iter"].toInt();
  m_slam_params->fmax_px_precision_           = (*vslam_set)["fmax_px_precision"].toFloat();
  m_slam_params->fmax_fbklt_dist_             = (*vslam_set)["fmax_fbklt_dist"].toFloat();
  m_slam_params->nklt_err_                    = (*vslam_set)["nklt_err"].toInt();
  m_slam_params->bdo_track_localmap_          = (*vslam_set)["bdo_track_localmap"].toInt() > 0;
  m_slam_params->fmax_desc_dist_              = (*vslam_set)["fmax_desc_dist"].toFloat();
  m_slam_params->fmax_proj_pxdist_            = (*vslam_set)["fmax_proj_pxdist"].toFloat();
  m_slam_params->doepipolar_                  = (*vslam_set)["do_epipolar"].toInt() > 0;
  m_slam_params->dop3p_                       = (*vslam_set)["do_p3p"].toInt() > 0;
  m_slam_params->bdo_random                   = (*vslam_set)["do_random"].toInt() > 0;
  m_slam_params->nransac_iter_                = (*vslam_set)["nransac_iter"].toInt();
  m_slam_params->fransac_err_                 = (*vslam_set)["fransac_err"].toFloat();
  m_slam_params->fepi_th_                     = m_slam_params->fransac_err_;
  m_slam_params->fmax_reproj_err_             = (*vslam_set)["fmax_reproj_err"].toFloat();
  m_slam_params->buse_inv_depth_              = (*vslam_set)["nransac_iter"].toInt() > 0;
  m_slam_params->robust_mono_th_              = (*vslam_set)["robust_mono_th"].toFloat();
  m_slam_params->use_sparse_schur_            = (*vslam_set)["use_sparse_schur"].toInt() > 0;
  m_slam_params->use_dogleg_                  = (*vslam_set)["use_dogleg"].toInt() > 0;
  m_slam_params->use_subspace_dogleg_         = (*vslam_set)["use_subspace_dogleg"].toInt() > 0;
  m_slam_params->use_nonmonotic_step_         = (*vslam_set)["use_nonmonotic_step"].toInt() > 0;
  m_slam_params->apply_l2_after_robust_       = (*vslam_set)["apply_l2_after_robust"].toInt() > 0;
  m_slam_params->nmin_covscore_               = (*vslam_set)["nmin_covscore"].toInt();
  m_slam_params->fkf_filtering_ratio_         = (*vslam_set)["fkf_filtering_ratio"].toFloat();
  m_slam_params->do_full_ba_                  = (*vslam_set)["do_full_ba"].toInt() > 0;

  m_slam_params->cam_left_model_ = "pinhole";
  m_slam_params->img_left_w_     = (*cam_set)["width"].toDouble();
  m_slam_params->img_left_h_    = (*cam_set)["height"].toDouble();
  m_slam_params->fxl_ = (*cam_set)["fx"].toDouble();
  m_slam_params->fyl_ = (*cam_set)["fy"].toDouble();
  m_slam_params->cxl_ = (*cam_set)["cx"].toDouble();
  m_slam_params->cyl_ = (*cam_set)["cy"].toDouble();
  m_slam_params->k1l_ = (*cam_set)["k1"].toDouble();
  m_slam_params->k2l_ = (*cam_set)["k2"].toDouble();
  m_slam_params->p1l_ = (*cam_set)["p1"].toDouble();
  m_slam_params->p2l_ = (*cam_set)["p2"].toDouble();

  double nbwcells = ceil( m_slam_params->img_left_w_ / (double)m_slam_params->nmaxdist_ );
  double nbhcells = ceil( m_slam_params->img_left_h_ / (double)m_slam_params->nmaxdist_ );
  m_slam_params->nbmaxkps_ = static_cast<int>(nbwcells * nbhcells);

  m_slam.reset(new SlamManager(m_slam_params));
}

Ov2Slam::~Ov2Slam()
{

}

void Ov2Slam::reset()
{
  m_slam->reset();
  m_id_previous = -1;
}

void Ov2Slam::close()
{
}

VisualSlamIF::State Ov2Slam::track(Frame::Ptr &frame, const cv::Mat &T_c2w_initial)
{
  // Set image resizing accoring to settings
  frame->setImageResizeFactor(m_resizing);

  const double timestamp = static_cast<double>(frame->getTimestamp())/10e3;
  LOG_IF_F(INFO, true, "Time stamp of frame: %4.2f [s]", timestamp);

  cv::Mat img = frame->getResizedImageRaw();

  if (img.channels() == 3)
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
  else if(img.channels() == 4)
    cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);

  m_slam->addNewMonoImage(timestamp, img);
  m_slam->spin();

  cv::Mat T_c2w = convertPose(m_slam->pcurframe_->Twc_.matrix3x4());

  if (m_slam_params->bvision_init_)
  {
    if (m_id_previous == -1)
    {
      frame->setVisualPose(T_c2w);
      frame->setSparseCloud(getTrackedMapPoints(), true);
      m_id_previous = m_slam->pcurframe_->kfid_;
      return State::INITIALIZED;
    }
    else if (m_id_previous < m_slam->pcurframe_->kfid_)
    {
      frame->setVisualPose(T_c2w);
      frame->setSparseCloud(getTrackedMapPoints(), true);
      m_id_previous = m_slam->pcurframe_->kfid_;
      return State::KEYFRAME_INSERT;
    }
    else
    {
      frame->setVisualPose(T_c2w);
      frame->setSparseCloud(getTrackedMapPoints(), true);
      return State::FRAME_INSERT;
    }
  }

  return State::LOST;
}

bool Ov2Slam::drawTrackedImage(cv::Mat &img) const
{
  img = m_slam->drawFrame();
  return !img.empty();
}

void Ov2Slam::printSettingsToLog()
{

}

cv::Mat Ov2Slam::convertPose(const Eigen::Matrix<double, 3, 4> &mat_eigen)
{
  cv::Mat mat_cv = (cv::Mat_<double>(3, 4) <<
    mat_eigen(0, 0), mat_eigen(0, 1), mat_eigen(0, 2), mat_eigen(0, 3),
    mat_eigen(1, 0), mat_eigen(1, 1), mat_eigen(1, 2), mat_eigen(1, 3),
    mat_eigen(2, 0), mat_eigen(2, 1), mat_eigen(2, 2), mat_eigen(2, 3));
  return mat_cv;
}

PointCloud::Ptr Ov2Slam::getTrackedMapPoints()
{
  std::vector<uint32_t> point_ids;
  cv::Mat points;

  std::unordered_map<int, Keypoint> keypoints = m_slam->pcurframe_->mapkps_;
  points.reserve(keypoints.size());

  for (const auto& kpt : keypoints)
  {
     std::shared_ptr<MapPoint> point_o2v = m_slam->pmap_->getMapPoint(kpt.first);

     if (point_o2v != nullptr && point_o2v->is3d_ && !point_o2v->isBad())
     {
       Eigen::Vector3d point_eigen = point_o2v->getPoint();
       cv::Mat point_cv = (cv::Mat_<double>(1, 3) << point_eigen.x(), point_eigen.y(), point_eigen.z());
       points.push_back(point_cv);

       point_ids.push_back(static_cast<uint32_t>(point_o2v->lmid_));
     }
  }

  return std::make_shared<PointCloud>(point_ids, points);
}
