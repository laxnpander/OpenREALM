/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <realm_types/camera.h>

namespace realm
{
namespace camera
{

Pinhole::Pinhole()
{
  reset();
}

// CONSTRUCTION

Pinhole::Pinhole(double fx,
                 double fy,
                 double cx,
                 double cy,
                 uint32_t img_width,
                 uint32_t img_height)
    : _img_width(img_width), _img_height(img_height)
{
  _K = cv::Mat_<double>(3, 3);
  _K.at<double>(0, 0) = fx;
  _K.at<double>(0, 1) = 0;
  _K.at<double>(0, 2) = cx;
  _K.at<double>(1, 0) = 0;
  _K.at<double>(1, 1) = fy;
  _K.at<double>(1, 2) = cy;
  _K.at<double>(2, 0) = 0;
  _K.at<double>(2, 1) = 0;
  _K.at<double>(2, 2) = 1.0;
}

Pinhole::Pinhole(const cv::Mat &K,
                 uint32_t img_width,
                 uint32_t img_height)
    : _K(K), _img_width(img_width), _img_height(img_height)
{
}

Pinhole::Pinhole(const cv::Mat &K,
                 const cv::Mat &dist_coeffs,
                 uint32_t img_width,
                 uint32_t img_height)
    : _K(K), _img_width(img_width), _img_height(img_height)
{
  setDistortionMap(dist_coeffs);
}

Pinhole::Pinhole(const Pinhole &that)
{
  _do_undistort = that._do_undistort;
  _R = that._R.clone();
  _t = that._t.clone();
  _K = that._K.clone();
  _dist_coeffs = that._dist_coeffs.clone();
  _undist_map1 = that._undist_map1.clone();
  _undist_map2 = that._undist_map2.clone();
  _img_width = that._img_width;
  _img_height = that._img_height;
}

Pinhole& Pinhole::operator=(const Pinhole &that)
{
  if (this != &that)
  {
    _do_undistort = that._do_undistort;
    _R = that._R.clone();
    _t = that._t.clone();
    _K = that._K.clone();
    _dist_coeffs = that._dist_coeffs.clone();
    _undist_map1 = that._undist_map1.clone();
    _undist_map2 = that._undist_map2.clone();
    _img_width = that._img_width;
    _img_height = that._img_height;
  }
  return *this;
}

bool Pinhole::isDistorted() const
{
  return _do_undistort;
}

uint32_t Pinhole::width() const
{
  return _img_width;
}

uint32_t Pinhole::height() const
{
  return _img_height;
}

double Pinhole::fx() const
{
  assert(!_K.empty() && _K.type() == CV_64F);
  return _K.at<double>(0, 0);
}

double Pinhole::fy() const
{
  assert(!_K.empty() && _K.type() == CV_64F);
  return _K.at<double>(1, 1);
}

double Pinhole::cx() const
{
  assert(!_K.empty() && _K.type() == CV_64F);
  return _K.at<double>(0, 2);
}

double Pinhole::cy() const
{
  assert(!_K.empty() && _K.type() == CV_64F);
  return _K.at<double>(1, 2);
}

double Pinhole::k1() const
{
  assert(!_dist_coeffs.empty() && _dist_coeffs.type() == CV_64F);
  return _dist_coeffs.at<double>(0);
}

double Pinhole::k2() const
{
  assert(!_dist_coeffs.empty() && _dist_coeffs.type() == CV_64F);
  return _dist_coeffs.at<double>(1);
}

double Pinhole::p1() const
{
  assert(!_dist_coeffs.empty() && _dist_coeffs.type() == CV_64F);
  return _dist_coeffs.at<double>(2);
}

double Pinhole::p2() const
{
  assert(!_dist_coeffs.empty() && _dist_coeffs.type() == CV_64F);
  return _dist_coeffs.at<double>(3);
}

cv::Mat Pinhole::K() const
{
  assert(!_K.empty() && _K.type() == CV_64F);
  return _K.clone();
}

cv::Mat Pinhole::P() const
{
  assert(!_K.empty() && _K.type() == CV_64F);
  assert(!_R.empty() && _R.type() == CV_64F);
  assert(!_t.empty() && _t.type() == CV_64F);
  return _K* Tw2c().rowRange(0, 3).colRange(0, 4);
}

cv::Mat Pinhole::distCoeffs() const
{
  assert(!_dist_coeffs.empty() && _K.type() == CV_64F);

  return _dist_coeffs.clone();
}

cv::Mat Pinhole::R() const
{
  assert(!_R.empty() && _R.type() == CV_64F);

  return _R.clone();
}

Eigen::Quaterniond Pinhole::orientation() const
{
  assert(!_R.empty() && _R.type() == CV_64F);

  Eigen::Matrix3d R_eigen;
  R_eigen << _R.at<double>(0, 0), _R.at<double>(0, 1), _R.at<double>(0, 2),
             _R.at<double>(1, 0), _R.at<double>(1, 1), _R.at<double>(1, 2),
             _R.at<double>(2, 0), _R.at<double>(2, 1), _R.at<double>(2, 2);
  return Eigen::Quaterniond(R_eigen);
}

cv::Mat Pinhole::t() const
{
  assert(!_t.empty() && _t.type() == CV_64F);
  return _t.clone();
}

cv::Mat Pinhole::pose() const
{
  if (_R.empty() || _t.empty())
    return cv::Mat();

  cv::Mat pose = cv::Mat_<double>(3, 4);
  _R.copyTo(pose.rowRange(0, 3).colRange(0, 3));
  _t.copyTo(pose.col(3));
  return std::move(pose);
}

cv::Mat Pinhole::Tw2c() const
{
  if (_R.empty() || _t.empty())
    return cv::Mat();

  cv::Mat T_w2c = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat R_w2c = _R.t();
  cv::Mat t_w2c = -R_w2c*_t;
  R_w2c.copyTo(T_w2c.rowRange(0, 3).colRange(0, 3));
  t_w2c.copyTo(T_w2c.rowRange(0, 3).col(3));
  return T_w2c;
}

// Pose of camera in the world reference
cv::Mat Pinhole::Tc2w() const
{
  if (_R.empty() || _t.empty())
    return cv::Mat();

  cv::Mat T_c2w = cv::Mat::eye(4, 4, CV_64F);
  _R.copyTo(T_c2w.rowRange(0, 3).colRange(0, 3));
  _t.copyTo(T_c2w.rowRange(0, 3).col(3));
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
  _dist_coeffs = dist_coeffs;
  // Set undistortion map
  // TODO: numeric limit
  if (isDistortionNonZero(_dist_coeffs))
  {
    cv::initUndistortRectifyMap(_K,
                                _dist_coeffs,
                                cv::Mat_<double>::eye(3, 3),
                                _K,
                                cv::Size(_img_width, _img_height),
                                CV_16SC2,
                                _undist_map1,
                                _undist_map2);
    _do_undistort = true;
  }
}

void Pinhole::setPose(const cv::Mat &pose)
{
  assert(!pose.empty() && pose.type() == CV_64F);
  assert(pose.rows == 3 && pose.cols == 4);
  _t = pose.col(3).rowRange(0, 3);
  _R = pose.colRange(0, 3).rowRange(0, 3);
}

Pinhole Pinhole::resize(double factor) const
{
  assert(!_K.empty());
  cv::Mat K = _K.clone();
  K.at<double>(0, 0) *= factor;
  K.at<double>(1, 1) *= factor;
  K.at<double>(0, 2) *= factor;
  K.at<double>(1, 2) *= factor;
  cv::Mat dist_coeffs = _dist_coeffs.clone();
  auto width = static_cast<uint32_t>(std::round((double)_img_width * factor));
  auto height = static_cast<uint32_t>(std::round((double)_img_height * factor));

  Pinhole cam_resized(K, dist_coeffs, width, height);
  if (!_R.empty() && !_t.empty())
  {
    cam_resized._R = _R.clone();
    cam_resized._t = _t.clone();
  }

  return cam_resized;
}

// FUNCTIONALITIES

cv::Mat Pinhole::computeImageBounds2Ddistorted() const
{
  return (cv::Mat_<double>(4, 2)
      << 0, 0, 0, (double) _img_height, (double) _img_width, (double) _img_height, (double) _img_width, 0);
}

cv::Mat Pinhole::computeImageBounds2D() const
{
  cv::Mat img_bounds = computeImageBounds2Ddistorted();
  img_bounds = img_bounds.reshape(2);
  cv::undistortPoints(img_bounds, img_bounds, _K, _dist_coeffs, cv::Mat(), _K);
  img_bounds = img_bounds.reshape(1);
  return img_bounds;
}

cv::Mat Pinhole::computeImageBoundsInWorld(const double &depth) const
{
  cv::Mat img_bounds = computeImageBounds2D();
  cv::Mat bounds_in_world;
  for (uint i = 0; i < 4; i++)
  {
    cv::Mat x = (cv::Mat_<double>(3, 1) << img_bounds.at<double>(i, 0), img_bounds.at<double>(i, 1), 1.0);
    cv::Mat pt = _R * (depth * _K.inv() * x) + _t;
    cv::Mat world_point = (cv::Mat_<double>(1, 3) << pt.at<double>(0), pt.at<double>(1), pt.at<double>(2));
    bounds_in_world.push_back(world_point);
  }
  return bounds_in_world;
}

cv::Mat Pinhole::projectImageBoundsToPlane(const cv::Mat &pt, const cv::Mat &n) const
{
  assert(!_R.empty() && !_t.empty() && !_K.empty());
  cv::Mat img_bounds = computeImageBounds2D();
  cv::Mat plane_points;
  for (uint i = 0; i < 4; i++)
  {
    cv::Mat u = (cv::Mat_<double>(3, 1) << img_bounds.at<double>(i, 0), img_bounds.at<double>(i, 1), 1.0);
    double s = (pt - _t).dot(n) / (_R * _K.inv() * u).dot(n);
    cv::Mat p = _R * (s * _K.inv() * u) + _t;
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
  if (_do_undistort)
    cv::remap(src, img_undistorted, _undist_map1, _undist_map2, interpolation);
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
  return cv::Rect2d(roi_l, roi_u, roi_r-roi_l, roi_u-roi_d);
}

cv::Mat Pinhole::unprojectPoint(double x, double y, double depth) const
{
  if (depth <= 0.0)
    return cv::Mat();
  cv::Mat point(4, 1, CV_64F);
  point.at<double>(0) = (x - _K.at<double>(0,2))*depth/_K.at<double>(0,0);
  point.at<double>(1) = (y - _K.at<double>(1,2))*depth/_K.at<double>(1,1);
  point.at<double>(2) = depth;
  point.at<double>(3) = 1;
  return Tc2w()*point;
}

// PRIVATE

void Pinhole::reset()
{
  _img_width = 0;
  _img_height = 0;
}

bool Pinhole::isDistortionNonZero(const cv::Mat &dist_coeffs)
{
  double sum = fabs(dist_coeffs.at<double>(0))+fabs(dist_coeffs.at<double>(1))
      +fabs(dist_coeffs.at<double>(2))+fabs(dist_coeffs.at<double>(3));
  return sum > 0.0001;
}

} // namespace camera

} // namespace realm