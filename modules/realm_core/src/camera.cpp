

#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>

// TODO: Update this to be conditional on OpenCV4
#include <opencv2/calib3d.hpp>

#include <realm_core/camera.h>

namespace realm
{
namespace camera
{

// CONSTRUCTION

Pinhole::Pinhole(double fx,
                 double fy,
                 double cx,
                 double cy,
                 uint32_t img_width,
                 uint32_t img_height)
    : m_do_undistort(false),
      m_width(img_width),
      m_height(img_height)
{
  m_K = cv::Mat_<double>(3, 3);
  m_K.at<double>(0, 0) = fx;
  m_K.at<double>(0, 1) = 0;
  m_K.at<double>(0, 2) = cx;
  m_K.at<double>(1, 0) = 0;
  m_K.at<double>(1, 1) = fy;
  m_K.at<double>(1, 2) = cy;
  m_K.at<double>(2, 0) = 0;
  m_K.at<double>(2, 1) = 0;
  m_K.at<double>(2, 2) = 1.0;
}

Pinhole::Pinhole(const cv::Mat &K,
                 uint32_t img_width,
                 uint32_t img_height)
    : m_do_undistort(false),
      m_K(K),
      m_width(img_width),
      m_height(img_height)
{
}

Pinhole::Pinhole(const cv::Mat &K,
                 const cv::Mat &dist_coeffs,
                 uint32_t img_width,
                 uint32_t img_height)
    : m_do_undistort(false),
      m_K(K),
      m_width(img_width),
      m_height(img_height)
{
  setDistortionMap(dist_coeffs);
}

Pinhole::Pinhole(const Pinhole &that)
: m_do_undistort(that.m_do_undistort),
  m_R(that.m_R.clone()),
  m_t(that.m_t.clone()),
  m_K(that.m_K.clone()),
  m_width(that.m_width),
  m_height(that.m_height)
{
  if (m_do_undistort)
  {
    m_dist_coeffs = that.m_dist_coeffs.clone();
    m_undistortion_map1 = that.m_undistortion_map1.clone();
    m_undistortion_map2 = that.m_undistortion_map2.clone();
  }
}

Pinhole& Pinhole::operator=(const Pinhole &that)
{
  if (this != &that)
  {
    m_do_undistort = that.m_do_undistort;
    m_R = that.m_R.clone();
    m_t = that.m_t.clone();
    m_K = that.m_K.clone();
    m_width = that.m_width;
    m_height = that.m_height;

    if (m_do_undistort) {
      m_dist_coeffs = that.m_dist_coeffs.clone();
      m_undistortion_map1 = that.m_undistortion_map1.clone();
      m_undistortion_map2 = that.m_undistortion_map2.clone();
    }
  }
  return *this;
}

bool Pinhole::hasDistortion() const
{
  return m_do_undistort;
}

uint32_t Pinhole::width() const
{
  return m_width;
}

uint32_t Pinhole::height() const
{
  return m_height;
}

double Pinhole::fx() const
{
  assert(!m_K.empty() && m_K.type() == CV_64F);
  return m_K.at<double>(0, 0);
}

double Pinhole::fy() const
{
  assert(!m_K.empty() && m_K.type() == CV_64F);
  return m_K.at<double>(1, 1);
}

double Pinhole::cx() const
{
  assert(!m_K.empty() && m_K.type() == CV_64F);
  return m_K.at<double>(0, 2);
}

double Pinhole::cy() const
{
  assert(!m_K.empty() && m_K.type() == CV_64F);
  return m_K.at<double>(1, 2);
}

double Pinhole::k1() const
{
  assert(!m_dist_coeffs.empty() && m_dist_coeffs.type() == CV_64F);
  return m_dist_coeffs.at<double>(0);
}

double Pinhole::k2() const
{
  assert(!m_dist_coeffs.empty() && m_dist_coeffs.type() == CV_64F);
  return m_dist_coeffs.at<double>(1);
}

double Pinhole::p1() const
{
  assert(!m_dist_coeffs.empty() && m_dist_coeffs.type() == CV_64F);
  return m_dist_coeffs.at<double>(2);
}

double Pinhole::p2() const
{
  assert(!m_dist_coeffs.empty() && m_dist_coeffs.type() == CV_64F);
  return m_dist_coeffs.at<double>(3);
}

double Pinhole::k3() const
{
  assert(!m_dist_coeffs.empty() && m_dist_coeffs.type() == CV_64F);
  return m_dist_coeffs.at<double>(4);
}

cv::Mat Pinhole::K() const
{
  assert(!m_K.empty() && m_K.type() == CV_64F);
  return m_K.clone();
}

cv::Mat Pinhole::P() const
{
  if (m_R.empty())
    throw(std::runtime_error("Error: Projection matrix could not be computed. Exterior rotation not set!"));
  if (m_t.empty())
    throw(std::runtime_error("Error: Projection matrix could not be computed. Exterior translation not set!"));

  cv::Mat T_w2c = Tw2c();

  cv::Mat P;
  hconcat(m_K * T_w2c.rowRange(0, 3).colRange(0, 3), m_K * T_w2c.rowRange(0, 3).col(3), P);
  return P;
}

cv::Mat Pinhole::distCoeffs() const
{
  assert(!m_dist_coeffs.empty() && m_K.type() == CV_64F);

  return m_dist_coeffs.clone();
}

cv::Mat Pinhole::R() const
{
  assert(!m_R.empty() && m_R.type() == CV_64F);

  return m_R.clone();
}

cv::Mat Pinhole::t() const
{
  assert(!m_t.empty() && m_t.type() == CV_64F);
  return m_t.clone();
}

cv::Mat Pinhole::pose() const
{
  if (m_R.empty() || m_t.empty())
    return cv::Mat();

  cv::Mat pose = cv::Mat_<double>(3, 4);
  m_R.copyTo(pose.rowRange(0, 3).colRange(0, 3));
  m_t.copyTo(pose.col(3));
  return std::move(pose);
}

cv::Mat Pinhole::Tw2c() const
{
  if (m_R.empty() || m_t.empty())
    return cv::Mat();

  cv::Mat T_w2c = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat R_w2c = m_R.t();
  cv::Mat t_w2c = -R_w2c * m_t;
  R_w2c.copyTo(T_w2c.rowRange(0, 3).colRange(0, 3));
  t_w2c.copyTo(T_w2c.rowRange(0, 3).col(3));
  return T_w2c;
}

// Pose of camera in the world reference
cv::Mat Pinhole::Tc2w() const
{
  if (m_R.empty() || m_t.empty())
    return cv::Mat();

  cv::Mat T_c2w = cv::Mat::eye(4, 4, CV_64F);
  m_R.copyTo(T_c2w.rowRange(0, 3).colRange(0, 3));
  m_t.copyTo(T_c2w.rowRange(0, 3).col(3));
  return T_c2w;
}

void Pinhole::setDistortionMap(const double &k1,
                               const double &k2,
                               const double &p1,
                               const double &p2,
                               const double &k3)
{
  cv::Mat dist_coeffs(1, 5, CV_64F);
  dist_coeffs.at<double>(0) = k1;
  dist_coeffs.at<double>(1) = k2;
  dist_coeffs.at<double>(2) = p1;
  dist_coeffs.at<double>(3) = p2;
  dist_coeffs.at<double>(4) = k3;
  // Set undistortion map
  setDistortionMap(dist_coeffs);
}

void Pinhole::setDistortionMap(const cv::Mat &dist_coeffs)
{
  assert(!dist_coeffs.empty() && dist_coeffs.type() == CV_64F);
  m_dist_coeffs = dist_coeffs;
  cv::initUndistortRectifyMap(m_K,
                              m_dist_coeffs,
                              cv::Mat_<double>::eye(3, 3),
                              m_K,
                              cv::Size(m_width, m_height),
                              CV_16SC2,
                              m_undistortion_map1,
                              m_undistortion_map2);
  m_do_undistort = true;
}

void Pinhole::setPose(const cv::Mat &pose)
{
  assert(!pose.empty() && pose.type() == CV_64F);
  assert(pose.rows == 3 && pose.cols == 4);
  m_t = pose.col(3).rowRange(0, 3);
  m_R = pose.colRange(0, 3).rowRange(0, 3);
}

Pinhole Pinhole::resize(double factor) const
{
  assert(!m_K.empty());
  cv::Mat K = m_K.clone();
  K.at<double>(0, 0) *= factor;
  K.at<double>(1, 1) *= factor;
  K.at<double>(0, 2) *= factor;
  K.at<double>(1, 2) *= factor;
  cv::Mat dist_coeffs = m_dist_coeffs.clone();
  auto width = static_cast<uint32_t>(std::round((double)m_width * factor));
  auto height = static_cast<uint32_t>(std::round((double)m_height * factor));

  Pinhole cam_resized(K, dist_coeffs, width, height);
  if (!m_R.empty() && !m_t.empty())
  {
    cam_resized.m_R = m_R.clone();
    cam_resized.m_t = m_t.clone();
  }

  return cam_resized;
}

Pinhole Pinhole::resize(uint32_t width, uint32_t height)
{
  if (width == 0 || height == 0)
    throw(std::invalid_argument("Error resizing camera: Resizing to zero dimensions not permitted."));

  double factor_x = static_cast<double>(width) / m_width;
  double factor_y = static_cast<double>(height) / m_height;

  if (fabs(factor_x - factor_y) > 10e-2)
    throw(std::invalid_argument("Error resizing camera: Only multiples of the original image size are supported."));

  return resize(factor_x);
}

// FUNCTIONALITIES

cv::Mat Pinhole::computeImageBounds2Ddistorted() const
{
  return (cv::Mat_<double>(4, 2)
      << 0, 0, 0, (double) m_height, (double) m_width, (double) m_height, (double) m_width, 0);
}

cv::Mat Pinhole::computeImageBounds2D() const
{
  cv::Mat img_bounds = computeImageBounds2Ddistorted();

  if (m_do_undistort)
  {
    img_bounds = img_bounds.reshape(2);
    cv::undistortPoints(img_bounds, img_bounds, m_K, m_dist_coeffs, cv::Mat(), m_K);
    img_bounds = img_bounds.reshape(1);
  }

  return img_bounds;
}

cv::Mat Pinhole::projectImageBoundsToPlane(const cv::Mat &pt, const cv::Mat &n) const
{
  assert(!m_R.empty() && !m_t.empty() && !m_K.empty());
  cv::Mat img_bounds = computeImageBounds2D();
  cv::Mat plane_points;
  for (uint i = 0; i < 4; i++)
  {
    cv::Mat u = (cv::Mat_<double>(3, 1) << img_bounds.at<double>(i, 0), img_bounds.at<double>(i, 1), 1.0);
    double s = (pt - m_t).dot(n) / (m_R * m_K.inv() * u).dot(n);
    cv::Mat p = m_R * (s * m_K.inv() * u) + m_t;
    cv::Mat world_point = (cv::Mat_<double>(1, 3) << p.at<double>(0), p.at<double>(1), p.at<double>(2));
    plane_points.push_back(world_point);
  }
  return plane_points;
}

cv::Mat Pinhole::undistort(const cv::Mat &src, int interpolation) const
{
  // If undistortion is not neccessary, just return input img
  // Elsewise undistort img
  cv::Mat img_undistorted;
  if (m_do_undistort)
    cv::remap(src, img_undistorted, m_undistortion_map1, m_undistortion_map2, interpolation);
  else
    img_undistorted = src;
  return img_undistorted;
}

cv::Rect2d Pinhole::projectImageBoundsToPlaneRoi(const cv::Mat &pt, const cv::Mat &n) const
{
  cv::Mat bounds_in_plane = projectImageBoundsToPlane(pt, n);
  double roi_l, roi_r;
  double roi_u, roi_d;
  cv::minMaxLoc(bounds_in_plane.col(0), &roi_l, &roi_r);
  cv::minMaxLoc(bounds_in_plane.col(1), &roi_d, &roi_u);
  return cv::Rect2d(roi_l, roi_d, roi_r-roi_l, roi_u-roi_d);
}

cv::Mat Pinhole::projectPointToWorld(double x, double y, double depth) const
{
  if (depth <= 0.0)
    return cv::Mat();
  cv::Mat point(4, 1, CV_64F);
  point.at<double>(0) = (x - m_K.at<double>(0, 2)) * depth / m_K.at<double>(0, 0);
  point.at<double>(1) = (y - m_K.at<double>(1, 2)) * depth / m_K.at<double>(1, 1);
  point.at<double>(2) = depth;
  point.at<double>(3) = 1;
  return Tc2w()*point;
}

} // namespace camera

} // namespace realm