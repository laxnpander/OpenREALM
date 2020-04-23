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

#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>

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
    : _do_undistort(false),
      _img_width(img_width),
      _img_height(img_height)
{
  _camera_matrix = cv::Mat_<double>(3, 3);
  _camera_matrix.at<double>(0, 0) = fx;
  _camera_matrix.at<double>(0, 1) = 0;
  _camera_matrix.at<double>(0, 2) = cx;
  _camera_matrix.at<double>(1, 0) = 0;
  _camera_matrix.at<double>(1, 1) = fy;
  _camera_matrix.at<double>(1, 2) = cy;
  _camera_matrix.at<double>(2, 0) = 0;
  _camera_matrix.at<double>(2, 1) = 0;
  _camera_matrix.at<double>(2, 2) = 1.0;
}

Pinhole::Pinhole(const cv::Mat &K,
                 uint32_t img_width,
                 uint32_t img_height)
    : _do_undistort(false),
      _camera_matrix(K),
      _img_width(img_width),
      _img_height(img_height)
{
}

Pinhole::Pinhole(const cv::Mat &K,
                 const cv::Mat &dist_coeffs,
                 uint32_t img_width,
                 uint32_t img_height)
    : _do_undistort(false),
      _camera_matrix(K),
      _img_width(img_width),
      _img_height(img_height)
{
  setDistortionMap(dist_coeffs);
}

Pinhole::Pinhole(const Pinhole &that)
: _do_undistort(that._do_undistort),
  _exterior_rotation(that._exterior_rotation.clone()),
  _exterior_translation(that._exterior_translation.clone()),
  _camera_matrix(that._camera_matrix.clone()),
  _img_width(that._img_width),
  _img_height(that._img_height)
{
  if (_do_undistort)
  {
    _distortion_coeff = that._distortion_coeff.clone();
    _undistortion_map1 = that._undistortion_map1.clone();
    _undistortion_map2 = that._undistortion_map2.clone();
  }
}

Pinhole& Pinhole::operator=(const Pinhole &that)
{
  if (this != &that)
  {
    _do_undistort = that._do_undistort;
    _exterior_rotation = that._exterior_rotation.clone();
    _exterior_translation = that._exterior_translation.clone();
    _camera_matrix = that._camera_matrix.clone();
    _img_width = that._img_width;
    _img_height = that._img_height;

    if (_do_undistort) {
      _distortion_coeff = that._distortion_coeff.clone();
      _undistortion_map1 = that._undistortion_map1.clone();
      _undistortion_map2 = that._undistortion_map2.clone();
    }
  }
  return *this;
}

bool Pinhole::hasDistortion() const
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
  assert(!_camera_matrix.empty() && _camera_matrix.type() == CV_64F);
  return _camera_matrix.at<double>(0, 0);
}

double Pinhole::fy() const
{
  assert(!_camera_matrix.empty() && _camera_matrix.type() == CV_64F);
  return _camera_matrix.at<double>(1, 1);
}

double Pinhole::cx() const
{
  assert(!_camera_matrix.empty() && _camera_matrix.type() == CV_64F);
  return _camera_matrix.at<double>(0, 2);
}

double Pinhole::cy() const
{
  assert(!_camera_matrix.empty() && _camera_matrix.type() == CV_64F);
  return _camera_matrix.at<double>(1, 2);
}

double Pinhole::k1() const
{
  assert(!_distortion_coeff.empty() && _distortion_coeff.type() == CV_64F);
  return _distortion_coeff.at<double>(0);
}

double Pinhole::k2() const
{
  assert(!_distortion_coeff.empty() && _distortion_coeff.type() == CV_64F);
  return _distortion_coeff.at<double>(1);
}

double Pinhole::p1() const
{
  assert(!_distortion_coeff.empty() && _distortion_coeff.type() == CV_64F);
  return _distortion_coeff.at<double>(2);
}

double Pinhole::p2() const
{
  assert(!_distortion_coeff.empty() && _distortion_coeff.type() == CV_64F);
  return _distortion_coeff.at<double>(3);
}

cv::Mat Pinhole::K() const
{
  assert(!_camera_matrix.empty() && _camera_matrix.type() == CV_64F);
  return _camera_matrix.clone();
}

cv::Mat Pinhole::P() const
{
  assert(!_camera_matrix.empty() && _camera_matrix.type() == CV_64F);
  assert(!_exterior_rotation.empty() && _exterior_rotation.type() == CV_64F);
  assert(!_exterior_translation.empty() && _exterior_translation.type() == CV_64F);
  return _camera_matrix * Tw2c().rowRange(0, 3).colRange(0, 4);
}

cv::Mat Pinhole::distCoeffs() const
{
  assert(!_distortion_coeff.empty() && _camera_matrix.type() == CV_64F);

  return _distortion_coeff.clone();
}

cv::Mat Pinhole::R() const
{
  assert(!_exterior_rotation.empty() && _exterior_rotation.type() == CV_64F);

  return _exterior_rotation.clone();
}

cv::Mat Pinhole::t() const
{
  assert(!_exterior_translation.empty() && _exterior_translation.type() == CV_64F);
  return _exterior_translation.clone();
}

cv::Mat Pinhole::pose() const
{
  if (_exterior_rotation.empty() || _exterior_translation.empty())
    return cv::Mat();

  cv::Mat pose = cv::Mat_<double>(3, 4);
  _exterior_rotation.copyTo(pose.rowRange(0, 3).colRange(0, 3));
  _exterior_translation.copyTo(pose.col(3));
  return std::move(pose);
}

cv::Mat Pinhole::Tw2c() const
{
  if (_exterior_rotation.empty() || _exterior_translation.empty())
    return cv::Mat();

  cv::Mat T_w2c = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat R_w2c = _exterior_rotation.t();
  cv::Mat t_w2c = -R_w2c * _exterior_translation;
  R_w2c.copyTo(T_w2c.rowRange(0, 3).colRange(0, 3));
  t_w2c.copyTo(T_w2c.rowRange(0, 3).col(3));
  return T_w2c;
}

// Pose of camera in the world reference
cv::Mat Pinhole::Tc2w() const
{
  if (_exterior_rotation.empty() || _exterior_translation.empty())
    return cv::Mat();

  cv::Mat T_c2w = cv::Mat::eye(4, 4, CV_64F);
  _exterior_rotation.copyTo(T_c2w.rowRange(0, 3).colRange(0, 3));
  _exterior_translation.copyTo(T_c2w.rowRange(0, 3).col(3));
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
  _distortion_coeff = dist_coeffs;
  cv::initUndistortRectifyMap(_camera_matrix,
                              _distortion_coeff,
                              cv::Mat_<double>::eye(3, 3),
                              _camera_matrix,
                              cv::Size(_img_width, _img_height),
                              CV_16SC2,
                              _undistortion_map1,
                              _undistortion_map2);
  _do_undistort = true;
}

void Pinhole::setPose(const cv::Mat &pose)
{
  assert(!pose.empty() && pose.type() == CV_64F);
  assert(pose.rows == 3 && pose.cols == 4);
  _exterior_translation = pose.col(3).rowRange(0, 3);
  _exterior_rotation = pose.colRange(0, 3).rowRange(0, 3);
}

Pinhole Pinhole::resize(double factor) const
{
  assert(!_camera_matrix.empty());
  cv::Mat K = _camera_matrix.clone();
  K.at<double>(0, 0) *= factor;
  K.at<double>(1, 1) *= factor;
  K.at<double>(0, 2) *= factor;
  K.at<double>(1, 2) *= factor;
  cv::Mat dist_coeffs = _distortion_coeff.clone();
  auto width = static_cast<uint32_t>(std::round((double)_img_width * factor));
  auto height = static_cast<uint32_t>(std::round((double)_img_height * factor));

  Pinhole cam_resized(K, dist_coeffs, width, height);
  if (!_exterior_rotation.empty() && !_exterior_translation.empty())
  {
    cam_resized._exterior_rotation = _exterior_rotation.clone();
    cam_resized._exterior_translation = _exterior_translation.clone();
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

  if (_do_undistort)
  {
    img_bounds = img_bounds.reshape(2);
    cv::undistortPoints(img_bounds, img_bounds, _camera_matrix, _distortion_coeff, cv::Mat(), _camera_matrix);
    img_bounds = img_bounds.reshape(1);
  }

  return img_bounds;
}

cv::Mat Pinhole::projectImageBoundsToPlane(const cv::Mat &pt, const cv::Mat &n) const
{
  assert(!_exterior_rotation.empty() && !_exterior_translation.empty() && !_camera_matrix.empty());
  cv::Mat img_bounds = computeImageBounds2D();
  cv::Mat plane_points;
  for (uint i = 0; i < 4; i++)
  {
    cv::Mat u = (cv::Mat_<double>(3, 1) << img_bounds.at<double>(i, 0), img_bounds.at<double>(i, 1), 1.0);
    double s = (pt - _exterior_translation).dot(n) / (_exterior_rotation * _camera_matrix.inv() * u).dot(n);
    cv::Mat p = _exterior_rotation * (s * _camera_matrix.inv() * u) + _exterior_translation;
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
    cv::remap(src, img_undistorted, _undistortion_map1, _undistortion_map2, interpolation);
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

cv::Mat Pinhole::projectPointToWorld(double x, double y, double depth) const
{
  if (depth <= 0.0)
    return cv::Mat();
  cv::Mat point(4, 1, CV_64F);
  point.at<double>(0) = (x - _camera_matrix.at<double>(0, 2)) * depth / _camera_matrix.at<double>(0, 0);
  point.at<double>(1) = (y - _camera_matrix.at<double>(1, 2)) * depth / _camera_matrix.at<double>(1, 1);
  point.at<double>(2) = depth;
  point.at<double>(3) = 1;
  return Tc2w()*point;
}

} // namespace camera

} // namespace realm