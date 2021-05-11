

#include <realm_vslam_base/geometric_referencer.h>
#include <fstream>

#include <eigen3/Eigen/Eigen>

using namespace realm;

GeometricReferencer::GeometricReferencer(double th_error, int min_nrof_frames)
: m_is_initialized(false),
  m_is_buisy(false),
  m_prev_nrof_unique(0),
  m_scale(0.0),
  m_th_error(th_error),
  m_error(0.0),
  m_min_nrof_frames(min_nrof_frames)
{

}

bool GeometricReferencer::isBuisy()
{
  std::unique_lock<std::mutex> lock(m_mutex_is_buisy);
  return m_is_buisy;
}

bool GeometricReferencer::isInitialized()
{
  std::unique_lock<std::mutex> lock(m_mutex_is_initialized);
  return m_is_initialized;
}

double GeometricReferencer::computeScaleChange(const Frame::Ptr &frame)
{
  if (!frame->isDepthComputed())
    return -1.0;

  SpatialMeasurement::Ptr s_curr = std::make_shared<SpatialMeasurement>();
  s_curr->first = frame->getDefaultPose();
  s_curr->second = frame->getVisualPose();

  int dit = 1;
  if (m_spatials.size() > 3)
    dit = m_spatials.size() / 3;

  std::vector<double> scales;
  for (int i = 0; i < m_spatials.size(); i+=dit)
  {
    SpatialMeasurement::Ptr s_ref = m_spatials[i];
    double scale = computeTwoPointScale(s_curr, s_ref, 0.02*frame->getMedianSceneDepth());

    if (scale > 0.0)
      scales.push_back(scale);
  }

  // Use only if there is more than one scale estimate
  if (scales.size() > 1)
  {
    double scale_avr = std::accumulate(scales.begin(), scales.end(), 0.0)/scales.size();

    // Compute scale difference in percent
    return fabs(1.0-scale_avr/m_scale)*100.0;
  }

  return -1.0;
}

void GeometricReferencer::setBuisy()
{
  std::unique_lock<std::mutex> lock(m_mutex_is_buisy);
  m_is_buisy = true;
}

void GeometricReferencer::setIdle()
{
  std::unique_lock<std::mutex> lock(m_mutex_is_buisy);
  m_is_buisy = false;
}

void GeometricReferencer::setReference(const cv::Mat &T_c2g)
{
  std::unique_lock<std::mutex> lock(m_mutex_t_c2g);
  m_transformation_w2g = T_c2g.clone();
}

void GeometricReferencer::init(const std::vector<Frame::Ptr> &frames)
{
  LOG_F(INFO, "Starting georeferencing...");

  if (isBuisy() || m_is_initialized)
  {
    LOG_F(INFO, "### GEOREFERENCE ABORTED ###");
    LOG_F(INFO, "Input frames: %lu", frames.size());
    LOG_IF_F(INFO, isBuisy(), "Georeferencer is buisy!");
    LOG_IF_F(INFO, m_is_initialized, "Georeferencer is initialized!");
    return;
  }

  setBuisy();

  // Identify valid measurements
  std::vector<Frame::Ptr> valid_input;
  for (const auto &f : frames)
    if (f->getSparseCloud() != nullptr && f->getSparseCloud()->size() > 50 && f->isDepthComputed())
    {
      valid_input.push_back(f);
    }

  if (valid_input.empty())
  {
    LOG_F(INFO, "### GEOREFERENCE ABORTED ###");
    LOG_F(INFO, "Valid frames: %lu. Sparse cloud empty!", valid_input.size());
    setIdle();
    return;
  }

  // Compute initial scale
  double vis_th = 0.02*valid_input[0]->getMedianSceneDepth();
  std::vector<double> scales;

  // Extract spatial informations from frames
  std::vector<SpatialMeasurement::Ptr> spatials;
  for (const auto &f : valid_input)
  {
    auto spatial = std::make_shared<SpatialMeasurement>();
    spatial->first = f->getDefaultPose();
    spatial->second = f->getPose();
    spatials.push_back(spatial);
  }

  // Extract measurements with unique GNSS and pose info
  std::vector<SpatialMeasurement::Ptr> unique_spatials;
  unique_spatials.push_back(spatials[0]);
  for (size_t i = 1; i < spatials.size(); i++)
  {
    SpatialMeasurement::Ptr spatial = spatials[i];
    bool is_unique = true;
    for (size_t j = unique_spatials.size(); j > 0; --j)
    {
      double scale = computeTwoPointScale(spatial, unique_spatials[j-1], vis_th);
      if (scale > 0.0)
        scales.push_back(scale);
      else
      {
        is_unique = false;
        break;
      }
    }
    if (is_unique)
      unique_spatials.push_back(spatial);
  }

  // Check if enough measurements and if more scales estimates than in the iteration before were computed
  if (unique_spatials.size() < m_min_nrof_frames || unique_spatials.size() == m_prev_nrof_unique)
  {
    LOG_F(INFO, "### GEOREFERENCE ABORTED ###");
    LOG_F(INFO, "Unique frames: %lu", unique_spatials.size());
    setIdle();
    return;
  }

  // Average scale and update member
  double scale_avr = std::accumulate(scales.begin(), scales.end(), 0.0)/scales.size();
  double dscale = scale_avr - m_scale;
  m_scale = scale_avr;
  m_prev_nrof_unique = unique_spatials.size();

  if (fabs(dscale) > m_th_error)
  {
    LOG_F(INFO, "### GEOREFERENCE ABORTED ###");
    LOG_F(INFO, "Scale change: %4.2f", fabs(dscale));
    setIdle();
    return;
  }

  LOG_F(INFO, "Proceeding georeferencing initial guess...");
  LOG_F(INFO, "Scale: %4.2f", scale_avr);

  cv::Mat T_p2g = cv::Mat::eye(4, 4, CV_64F);
  T_p2g.at<double>(0, 0) = scale_avr;
  T_p2g.at<double>(1, 1) = scale_avr;
  T_p2g.at<double>(2, 2) = scale_avr;
  cv::Mat T_c2g = refineReference(unique_spatials, T_p2g, 5.0);
  setReference(T_c2g);

  m_spatials = unique_spatials;

  std::unique_lock<std::mutex> lock(m_mutex_is_initialized);
  m_is_initialized = true;

  setIdle();
  LOG_F(INFO, "Finished georeference try!");
}

void GeometricReferencer::update(const Frame::Ptr &frame)
{
  if (isBuisy() || !frame->isDepthComputed())
    return;

  SpatialMeasurement::Ptr s_curr = std::make_shared<SpatialMeasurement>();
  s_curr->first = frame->getDefaultPose();
  s_curr->second = frame->getVisualPose();

  SpatialMeasurement::Ptr s_prev = m_spatials.back();

  setBuisy();

  if (computeTwoPointScale(s_curr, s_prev, 0.02*frame->getMedianSceneDepth()) > 0.0)
  {
    m_spatials.push_back(s_curr);
    cv::Mat T_c2g = refineReference(m_spatials, m_transformation_w2g.clone(), 3.0);
    setReference(T_c2g);

    double error = computeAverageReferenceError(m_spatials, T_c2g);
    double derror = fabs(error - m_error);
    m_error = error;

    double sx = cv::norm(T_c2g.col(0));
    double sy = cv::norm(T_c2g.col(1));;
    double sz = cv::norm(T_c2g.col(2));;

    LOG_F(INFO, "### GEOREFERENCE UPDATE ###");
    LOG_F(INFO, "Transformation matrix:");
    LOG_F(INFO, "%f %f %f %f", T_c2g.at<double>(0, 0), T_c2g.at<double>(0, 1), T_c2g.at<double>(0, 2), T_c2g.at<double>(0, 3));
    LOG_F(INFO, "%f %f %f %f", T_c2g.at<double>(1, 0), T_c2g.at<double>(1, 1), T_c2g.at<double>(1, 2), T_c2g.at<double>(1, 3));
    LOG_F(INFO, "%f %f %f %f", T_c2g.at<double>(2, 0), T_c2g.at<double>(2, 1), T_c2g.at<double>(2, 2), T_c2g.at<double>(2, 3));
    LOG_F(INFO, "%f %f %f %f", T_c2g.at<double>(3, 0), T_c2g.at<double>(3, 1), T_c2g.at<double>(3, 2), T_c2g.at<double>(3, 3));
    LOG_F(INFO, "Error: %4.2f", error);
    LOG_F(INFO, "dError: %4.2f", derror);
    LOG_F(INFO, "Scale (sx, sy, sz): (%4.2f, %4.2f, %4.2f)", sx, sy, sz);
  }
  setIdle();
}

cv::Mat GeometricReferencer::getTransformation()
{
  std::unique_lock<std::mutex> lock(m_mutex_t_c2g);
  return m_transformation_w2g.clone();
}

double GeometricReferencer::computeTwoPointScale(const SpatialMeasurement::Ptr &s1, const SpatialMeasurement::Ptr &s2, double th_visual)
{
  // First create mat with positions x,y,z
  double f1_gis[3] = {s1->first.at<double>(0, 3), s1->first.at<double>(1, 3), s1->first.at<double>(2, 3)};
  double f1_vis[3] = {s1->second.at<double>(0, 3), s1->second.at<double>(1, 3), s1->second.at<double>(2, 3)};
  double f2_gis[3] = {s2->first.at<double>(0, 3), s2->first.at<double>(1, 3), s2->first.at<double>(2, 3)};
  double f2_vis[3] = {s2->second.at<double>(0, 3), s2->second.at<double>(1, 3), s2->second.at<double>(2, 3)};

  // Calculate vector length
  double dist_v = sqrt(pow(f2_vis[0]-f1_vis[0], 2)+pow(f2_vis[1]-f1_vis[1], 2)+pow(f2_vis[2]-f1_vis[2], 2));
  double dist_g = sqrt(pow(f2_gis[0]-f1_gis[0], 2)+pow(f2_gis[1]-f1_gis[1], 2)+pow(f2_gis[2]-f1_gis[2], 2));

  // Check if legit scale mmt
  if (dist_g > 10.0 && dist_v > th_visual)
    return dist_g/dist_v; // Scale
  else
    return -1.0;          // Invalid Value
}

cv::Mat GeometricReferencer::refineReference(const std::vector<SpatialMeasurement::Ptr> &spatials, const cv::Mat &T_c2w, double z_weight)
{
  // First define basic eigen variables
  size_t nrof_points = spatials.size()*4;
  Eigen::Matrix< Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > src_points(3, nrof_points);
  Eigen::Matrix< Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > dst_points(3, nrof_points);

  int i, j;
  cv::Mat hom_row = (cv::Mat_<double>(1, 4) << 0.0, 0.0, 0.0, 1.0);
  for (i = 0, j = 0; i < spatials.size(); ++i, j+=4)
  {
    cv::Mat T_c2w_gis = spatials[i]->first.clone();
    T_c2w_gis.push_back(hom_row);

    cv::Mat e_gis_0, e_gis_x, e_gis_y, e_gis_z;
    try
    {
      e_gis_0 = applyTransformation(T_c2w_gis, (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0));
      e_gis_x = applyTransformation(T_c2w_gis, (cv::Mat_<double>(3, 1) << 1.0, 0.0, 0.0));
      e_gis_y = applyTransformation(T_c2w_gis, (cv::Mat_<double>(3, 1) << 0.0, 1.0, 0.0));
      e_gis_z = applyTransformation(T_c2w_gis, (cv::Mat_<double>(3, 1) << 0.0, 0.0, z_weight*1.0));
    }
    catch(std::invalid_argument &e)
    {
      LOG_F(ERROR, "%s", e.what());
      continue;
    }

    dst_points.col(j  ) << e_gis_0.at<double>(0), e_gis_0.at<double>(1), e_gis_0.at<double>(2);
    dst_points.col(j+1) << e_gis_x.at<double>(0), e_gis_x.at<double>(1), e_gis_x.at<double>(2);
    dst_points.col(j+2) << e_gis_y.at<double>(0), e_gis_y.at<double>(1), e_gis_y.at<double>(2);
    dst_points.col(j+3) << e_gis_z.at<double>(0), e_gis_z.at<double>(1), e_gis_z.at<double>(2);

    cv::Mat T_c2w_vis = spatials[i]->second.clone();
    T_c2w_vis.push_back(hom_row);
    T_c2w_vis = T_c2w * T_c2w_vis;

    // Remove scale from R
    T_c2w_vis.rowRange(0, 3).col(0) /= cv::norm(T_c2w_vis.rowRange(0, 3).col(0));
    T_c2w_vis.rowRange(0, 3).col(1) /= cv::norm(T_c2w_vis.rowRange(0, 3).col(1));
    T_c2w_vis.rowRange(0, 3).col(2) /= cv::norm(T_c2w_vis.rowRange(0, 3).col(2));

    cv::Mat e_vis_0, e_vis_x, e_vis_y, e_vis_z;
    try
    {
      e_vis_0 = applyTransformation(T_c2w_vis, (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0));
      e_vis_x = applyTransformation(T_c2w_vis, (cv::Mat_<double>(3, 1) << 1.0, 0.0, 0.0));
      e_vis_y = applyTransformation(T_c2w_vis, (cv::Mat_<double>(3, 1) << 0.0, 1.0, 0.0));
      e_vis_z = applyTransformation(T_c2w_vis, (cv::Mat_<double>(3, 1) << 0.0, 0.0, z_weight*1.0));
    }
    catch(std::invalid_argument &e)
    {
      LOG_F(ERROR, "%s", e.what());
      continue;
    }

    src_points.col(j  ) << e_vis_0.at<double>(0), e_vis_0.at<double>(1), e_vis_0.at<double>(2);
    src_points.col(j+1) << e_vis_x.at<double>(0), e_vis_x.at<double>(1), e_vis_x.at<double>(2);
    src_points.col(j+2) << e_vis_y.at<double>(0), e_vis_y.at<double>(1), e_vis_y.at<double>(2);
    src_points.col(j+3) << e_vis_z.at<double>(0), e_vis_z.at<double>(1), e_vis_z.at<double>(2);
  }

  Eigen::Matrix4d T_refine_eigen = Eigen::umeyama(src_points, dst_points, true);

  cv::Mat T_refine_cv(4, 4, CV_64F);
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      T_refine_cv.at<double>(r, c) = T_refine_eigen(r, c);

  return T_refine_cv * T_c2w;
}

double GeometricReferencer::computeAverageReferenceError(const std::vector<SpatialMeasurement::Ptr> &spatials, const cv::Mat &T_c2w)
{
  double error = 0.0;
  for (const auto &s : spatials)
  {
    cv::Mat pose_gis = s->first;
    cv::Mat pose_vis = s->second;

    cv::Mat pt_vis;
    try
    {
      pt_vis = applyTransformation(T_c2w, pose_vis.rowRange(0, 3).col(3));
    }
    catch(std::invalid_argument &e)
    {
      LOG_F(ERROR, "%s", e.what());
      continue;
    }
    cv::Mat pt_gis = pose_gis.rowRange(0, 3).col(3);
    error += cv::norm(pt_vis - pt_gis);
  }
  error /= (double)spatials.size();
  return error;
}

cv::Mat GeometricReferencer::applyTransformation(const cv::Mat &T, const cv::Mat &pt)
{
  cv::Mat pt_hom;
  if (pt.rows == 3 && pt.cols == 1)
    pt_hom = pt.clone();
  else if (pt.rows == 1 && pt.cols == 3)
    pt_hom = pt.t();
  else
    throw(std::invalid_argument("Error applying transformation: Point has wrong dimensions." ));
  pt_hom.push_back(1.0);
  cv::Mat pt_t = (T * pt_hom);
  pt_t *= 1/pt_t.at<double>(3);
  return (pt_t.rowRange(0, 3));

}