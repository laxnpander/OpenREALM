

#include <realm_densifier_base/plane_sweep.h>

#include <psl/exception.h>

using namespace realm;
using namespace densifier;

PlaneSweep::PlaneSweep(const DensifierSettings::Ptr &settings)
: m_nrof_frames((uint8_t)(*settings)["n_cams"].toInt()), m_resizing((*settings)["resizing"].toDouble())
{
  assert(m_nrof_frames > 1);
  settings->print();
  m_settings.enable_subpix            = (*settings)["enable_subpix"].toInt() > 0;
  m_settings.enable_color_match       = (*settings)["enable_color_match"].toInt() > 0;
  m_settings.enable_out_best_depth    = (*settings)["enable_out_best_depth"].toInt() > 0;
  m_settings.enable_out_best_cost     = (*settings)["enable_out_best_cost"].toInt() > 0;
  m_settings.enable_out_cost_vol      = (*settings)["enable_out_cost_vol"].toInt() > 0;
  m_settings.enable_out_uniq_ratio    = (*settings)["enable_out_uniq_ratio"].toInt() > 0;
  m_settings.nrof_planes              =  (*settings)["nrof_planes"].toInt();
  m_settings.scale                    =  (*settings)["scale"].toDouble();
  m_settings.match_window_size.width  =  (*settings)["match_window_size_x"].toInt();
  m_settings.match_window_size.height =  (*settings)["match_window_size_y"].toInt();
  m_settings.occlusion_mode           = (PSL::PlaneSweepOcclusionMode)       (*settings)["occlusion_mode"].toInt();
  m_settings.plane_gen_mode           = (PSL::PlaneSweepPlaneGenerationMode) (*settings)["plane_gen_mode"].toInt();
  m_settings.match_cost               = (PSL::PlaneSweepMatchingCosts)       (*settings)["match_cost"].toInt();
  m_settings.subpx_interp_mode        = (PSL::PlaneSweepSubPixelInterpMode)  (*settings)["subpx_interp_mode"].toInt();
}

Depthmap::Ptr PlaneSweep::densify(const std::deque<Frame::Ptr> &frames, uint8_t ref_idx)
{
  assert(frames.size() == m_nrof_frames);

  // Get min and max scene depth for reference frame (middle frame)
  Frame::Ptr frame_ref = frames[ref_idx];
  auto min_depth = (float) frame_ref->getMinSceneDepth();
  auto max_depth = (float) frame_ref->getMaxSceneDepth();

  PSL::CudaPlaneSweep cps;
  cps.setScale(m_settings.scale);
  cps.setMatchWindowSize(m_settings.match_window_size.width, m_settings.match_window_size.height);
  cps.setNumPlanes(m_settings.nrof_planes);
  cps.setOcclusionMode(m_settings.occlusion_mode);
  cps.setPlaneGenerationMode(m_settings.plane_gen_mode);
  cps.setMatchingCosts(m_settings.match_cost);
  cps.setSubPixelInterpolationMode(m_settings.subpx_interp_mode);
  cps.enableOutputBestCosts(m_settings.enable_out_best_cost);
  cps.enableOuputUniquenessRatio(m_settings.enable_out_uniq_ratio);
  cps.enableOutputCostVolume(m_settings.enable_out_cost_vol);
  cps.setZRange(min_depth, max_depth);

  if (m_settings.enable_out_best_cost)
    cps.enableOutputBestDepth();
  if (m_settings.enable_color_match)
    cps.enableColorMatching();
  if (m_settings.enable_color_match)
    cps.enableSubPixel();

  // Create psl cameras for densification and feed to plane sweep handle
  int ref_id_psl = 0;
  for (uint32_t i = 0; i < frames.size(); ++i)
  {
    // Get frame from container and prepare
    Frame::Ptr frame = frames[i];
    frame->setImageResizeFactor(m_resizing);

    // Use resized image grayscale
    cv::Mat img = frame->getResizedImageUndistorted();
    cv::Mat img_valid = fixImageType(img);
    //cv::cvtColor(img, img, CV_BGR2GRAY);

    // Convert to PSL style camera
    PSL::CameraMatrix<double> cam = convertToPslCamera(frames[i]->getResizedCamera());

    // Feed image to plane sweep handle and safe id
    int id = cps.addImage(img_valid, cam);

    // Reference idx will be used to identify reference frame
    if (i == ref_idx)
      ref_id_psl = id;
  }

  // Now start processing for reference frame
  try
  {
    cps.process(ref_id_psl);
  }
  catch (const PSL::Exception &e)
  {
    LOG_F(WARNING, "Densification failed due to exception: %s", e.what());
    return nullptr;
  }

  // Get depthmap
  PSL::DepthMap<float, double> depth_map_psl = cps.getBestDepth();

  return std::make_shared<Depthmap>(convertToCvMat(depth_map_psl), *frame_ref->getResizedCamera());
}

uint8_t PlaneSweep::getNrofInputFrames()
{
  return m_nrof_frames;
}

double PlaneSweep::getResizeFactor()
{
  return m_resizing;
}

cv::Mat PlaneSweep::convertToCvMat(PSL::DepthMap<float, double> &depth_map_psl)
{
  uint32_t height = depth_map_psl.getHeight();
  uint32_t width = depth_map_psl.getWidth();
  cv::Mat_<float> depths_mat(height, width, depth_map_psl.getDataPtr());
  depths_mat.convertTo(depths_mat, CV_32F);
  return depths_mat.clone();
}

cv::Mat PlaneSweep::fixImageType(const cv::Mat &img)
{
  cv::Mat img_fixed;
  if (m_settings.enable_color_match)
  {
    if (img.channels() == 1)
      throw(std::invalid_argument("Error fixing image type: Image has only one channel, no color matching possible!"));
    else
      img_fixed = img;
  }
  else
  {
    if (img.channels() == 4)
      cv::cvtColor(img, img_fixed, cv::COLOR_BGRA2BGR);
    else
      img_fixed = img;
  }
  return img_fixed;
}

PSL::CameraMatrix<double> PlaneSweep::convertToPslCamera(const camera::Pinhole::Ptr &cam)
{
  // Convert calibration
  cv::Mat cv_K = cam->K();
  Eigen::Matrix<double, 3, 3> K = Eigen::Matrix3d::Identity();
  K(0, 0) = cv_K.at<double>(0, 0);
  K(1, 1) = cv_K.at<double>(1, 1);
  K(0, 2) = cv_K.at<double>(0, 2);
  K(1, 2) = cv_K.at<double>(1, 2);

  // Transformation defined as camera to world
  cv::Mat cv_T_w2c = cam->Tw2c();

  // Convert rotation mat
  cv::Mat cv_R = cv_T_w2c.rowRange(0, 3).colRange(0, 3);
  Eigen::Matrix<double, 3, 3> R = Eigen::Matrix3d(3, 3);
  R << cv_R.at<double>(0, 0), cv_R.at<double>(0, 1), cv_R.at<double>(0, 2),
       cv_R.at<double>(1, 0), cv_R.at<double>(1, 1), cv_R.at<double>(1, 2),
       cv_R.at<double>(2, 0), cv_R.at<double>(2, 1), cv_R.at<double>(2, 2);

  // Convert translation vec
  cv::Mat cv_t = cv_T_w2c.rowRange(0, 3).col(3);
  Eigen::Matrix<double, 3, 1> t;
  t << cv_t.at<double>(0), cv_t.at<double>(1), cv_t.at<double>(2);

  return PSL::CameraMatrix<double>(K, R, t);
}

void PlaneSweep::printSettingsToLog()
{
  LOG_F(INFO, "### PlaneSweep settings ###");
  LOG_F(INFO, "- nframes: %i", m_nrof_frames);
  LOG_F(INFO, "- enable_subpix: %i", m_settings.enable_subpix);
  LOG_F(INFO, "- enable_color_match: %i", m_settings.enable_color_match);
  LOG_F(INFO, "- enable_out_best_depth: %i", m_settings.enable_out_best_depth);
  LOG_F(INFO, "- enable_out_best_cost: %i", m_settings.enable_out_best_cost);
  LOG_F(INFO, "- enable_out_cost_vol: %i", m_settings.enable_out_cost_vol);
  LOG_F(INFO, "- enable_out_uniq_ratio: %i", m_settings.enable_out_uniq_ratio);
  LOG_F(INFO, "- nrof_planes: %i", m_settings.nrof_planes);
  LOG_F(INFO, "- scale: %2.2f", m_settings.scale);
  LOG_F(INFO, "- match_window_size_width: %i", m_settings.match_window_size.width);
  LOG_F(INFO, "- match_window_size_height: %i", m_settings.match_window_size.height);
  LOG_F(INFO, "- occlusion_mode: %i", static_cast<int>(m_settings.occlusion_mode));
  LOG_F(INFO, "- plane_gen_mode: %i", static_cast<int>(m_settings.plane_gen_mode));
  LOG_F(INFO, "- match_cost: %i", static_cast<int>(m_settings.match_cost));
  LOG_F(INFO, "- subpx_interp_mode: %i", static_cast<int>(m_settings.subpx_interp_mode));


}
