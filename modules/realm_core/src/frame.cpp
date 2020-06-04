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

#include <iostream>
#include <realm_core/frame.h>

namespace realm
{

// CONSTRUCTION
Frame::Frame(const std::string &camera_id,
             const uint32_t &frame_id,
             const uint64_t &timestamp,
             const cv::Mat &img,
             const UTMPose &utm,
             const camera::Pinhole::Ptr &cam,
             const cv::Mat &orientation)
    : _camera_id(camera_id),
      _frame_id(frame_id),
      _is_keyframe(false),
      _is_georeferenced(false),
      _is_img_resizing_set(false),
      _is_depth_computed(false),
      _has_accurate_pose(false),
      _surface_assumption(SurfaceAssumption::PLANAR),
      _timestamp(timestamp),
      _img(img),
      _utm(utm),
      _camera_model(cam),
      _orientation(orientation),
      _img_resize_factor(0.0),
      _min_scene_depth(0.0),
      _max_scene_depth(0.0),
      _med_scene_depth(0.0)
{
  _camera_model->setPose(getDefaultPose());
}

// GETTER

std::string Frame::getCameraId() const
{
  return _camera_id;
}

uint32_t Frame::getFrameId() const
{
  return _frame_id;
}

uint32_t Frame::getResizedImageWidth() const
{
  assert(_is_img_resizing_set);
  return (uint32_t)((double) _camera_model->width() * _img_resize_factor);
}

uint32_t Frame::getResizedImageHeight() const
{
  assert(_is_img_resizing_set);
  return (uint32_t)((double) _camera_model->height() * _img_resize_factor);
}

double Frame::getMinSceneDepth() const
{
  assert(_is_depth_computed);
  return _min_scene_depth;
}

double Frame::getMaxSceneDepth() const
{
  assert(_is_depth_computed);
  return _max_scene_depth;
}

double Frame::getMedianSceneDepth() const
{
  assert(_is_depth_computed);
  return _med_scene_depth;
}

cv::Size Frame::getResizedImageSize() const
{
  assert(_is_img_resizing_set);
  auto width = (uint32_t)((double) _camera_model->width() * _img_resize_factor);
  auto height = (uint32_t)((double) _camera_model->height() * _img_resize_factor);
  return cv::Size(width, height);
}

cv::Mat Frame::getImageUndistorted() const
{
  cv::Mat img_undistorted;
  if(_camera_model->hasDistortion())
    img_undistorted = _camera_model->undistort(_img, CV_INTER_LINEAR);
  else
    img_undistorted = _img;
  return std::move(img_undistorted);
}

cv::Mat Frame::getImageRaw() const
{
  // - No deep copy
  return _img;
}

cv::Mat Frame::getDefaultPose() const
{
  // Translation set to measured utm coordinates
  cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
  t.at<double>(0) = _utm.easting;
  t.at<double>(1) = _utm.northing;
  t.at<double>(2) = _utm.altitude;

  // Create default pose
  cv::Mat default_pose = cv::Mat::eye(3, 4, CV_64F);
  _orientation.copyTo(default_pose.rowRange(0, 3).colRange(0, 3));
  t.col(0).copyTo(default_pose.col(3));
  return default_pose;
}

cv::Mat Frame::getResizedImageUndistorted() const
{
  // - Resized image will be calculated and set with first
  // access to avoid multiple costly resizing procedures
  // - Check also if resize factor has not changed in the
  // meantime
  // - No deep copy
  assert(_is_img_resizing_set);
  camera::Pinhole cam_resized = _camera_model->resize(_img_resize_factor);
  return cam_resized.undistort(_img_resized, CV_INTER_LINEAR);
}

cv::Mat Frame::getResizedImageRaw() const
{
  // deep copy, as it might be painted or modified
  assert(_is_img_resizing_set);
  return _img_resized.clone();
}

cv::Mat Frame::getResizedCalibration() const
{
  assert(_is_img_resizing_set);
  return _camera_model->resize(_img_resize_factor).K();
}

cv::Mat Frame::getSurfacePoints() const
{
  if (_surface_points.empty())
    return cv::Mat();
  else
    return _surface_points.clone();
}

cv::Mat Frame::getPose() const
{
  // Camera pose is a 3x4 motion matrix. However, it depends on the current state of information how it exactly looks like.
  // If frame pose is accurate, then two options exist:
  // 1) Pose is accurate and georeference was computed -> return motion in the geographic frame
  // 2) Pose is accurate but georeference was not computed -> return motion in visual world frame
  // If frame pose is not accurate, then return the default pose based on GNSS and heading

  if (hasAccuratePose())
  {
    // Option 1+2: Cameras pose is always uptodate
    return _camera_model->pose();
  }

  // Default:
  return getDefaultPose();
}

cv::Mat Frame::getVisualPose() const
{
  return _motion_c2w.clone();
}

cv::Mat Frame::getGeographicPose() const
{
  return _motion_c2g.clone();
}

cv::Mat Frame::getGeoreference() const
{
  return _transformation_w2g.clone();
}

SurfaceAssumption Frame::getSurfaceAssumption() const
{
  return _surface_assumption;
}

CvGridMap::Ptr Frame::getObservedMap() const
{
  assert(_observed_map != nullptr);
  return _observed_map;
}

UTMPose Frame::getGnssUtm() const
{
  return _utm;
}

camera::Pinhole::ConstPtr Frame::getCamera() const
{
  return _camera_model;
}

uint64_t Frame::getTimestamp() const
{
  // in [nanosec]
  return _timestamp;
}

camera::Pinhole::Ptr Frame::getResizedCamera() const
{
  assert(_is_img_resizing_set);
  return std::make_shared<camera::Pinhole>(_camera_model->resize(_img_resize_factor));
}

// SETTER

void Frame::setVisualPose(const cv::Mat &pose)
{
  _motion_c2w = pose;

  // If frame is already georeferenced, then set camera pose as geographic pose. Otherwise use visual pose
  if (_is_georeferenced)
  {
    cv::Mat M_c2g = applyTransformationToVisualPose(_transformation_w2g);
    setGeographicPose(M_c2g);
  }
  else
  {
    std::lock_guard<std::mutex> lock(_mutex_cam);
    _camera_model->setPose(pose);
  }

  setPoseAccurate(true);
}

void Frame::setGeographicPose(const cv::Mat &pose)
{
  if (!pose.empty())
  {
    std::lock_guard<std::mutex> lock(_mutex_cam);
    _camera_model->setPose(pose);
    _motion_c2g = pose;
    setPoseAccurate(true);
  }
}

void Frame::setGeoreference(const cv::Mat &T_w2g)
{
  if (T_w2g.empty())
    throw(std::invalid_argument("Error setting georeference: Transformation is empty!"));

  std::lock_guard<std::mutex> lock(_mutex_T_w2g);
  _transformation_w2g = T_w2g.clone();
  _is_georeferenced = true;
}

void Frame::setSurfacePoints(const cv::Mat &surface_pts)
{
  if (surface_pts.empty())
    return;

  _mutex_surface_pts.lock();
  _surface_points = surface_pts;
  _mutex_surface_pts.unlock();

  computeSceneDepth();
}

void Frame::setObservedMap(const CvGridMap::Ptr &observed_map)
{
  assert(!observed_map->empty());
  std::lock_guard<std::mutex> lock(_mutex_observed_map);
  _observed_map = observed_map;
}

void Frame::setKeyframe(bool flag)
{
  std::lock_guard<std::mutex> lock(_mutex_flags);
  _is_keyframe = flag;
}

void Frame::setPoseAccurate(bool flag)
{
  std::lock_guard<std::mutex> lock(_mutex_flags);
  _has_accurate_pose = flag;
}

void Frame::setSurfaceAssumption(SurfaceAssumption assumption)
{
  std::lock_guard<std::mutex> lock(_mutex_flags);
  _surface_assumption = assumption;
}

void Frame::setImageResizeFactor(const double &value)
{
  std::lock_guard<std::mutex> lock(_mutex_img_resized);
  _img_resize_factor = value;
  cv::resize(_img, _img_resized, cv::Size(), _img_resize_factor, _img_resize_factor);
  _is_img_resizing_set = true;
}


// FUNCTIONALITY

void Frame::initGeoreference(const cv::Mat &T)
{
  assert(!T.empty());

  _transformation_w2g = cv::Mat::eye(4, 4, CV_64F);
  updateGeoreference(T, true);
}

bool Frame::isKeyframe() const
{
  return _is_keyframe;
}

bool Frame::isGeoreferenced() const
{
  return _is_georeferenced;
}

bool Frame::isImageResizeSet() const
{
  return _is_img_resizing_set;
}

bool Frame::isDepthComputed() const
{
  return _is_depth_computed;
}

bool Frame::hasObservedMap() const
{
  return !(_observed_map == nullptr || _observed_map->empty());
}

bool Frame::hasAccuratePose() const
{
  return _has_accurate_pose;
}

std::string Frame::print()
{
  std::lock_guard<std::mutex> lock(_mutex_flags);
  char buffer[5000];
  sprintf(buffer, "### FRAME INFO ###\n");
  sprintf(buffer + strlen(buffer), "Stamp: %lu \n", _timestamp);
  sprintf(buffer + strlen(buffer), "Image: [%i x %i]\n", _img.cols, _img.rows);
  sprintf(buffer + strlen(buffer), "GNSS: [%4.02f E, %4.02f N, %4.02f Alt, %4.02f Head]\n",
          _utm.easting, _utm.northing, _utm.altitude, _utm.heading);
  sprintf(buffer + strlen(buffer), "Is key frame: %s\n", (_is_keyframe ? "yes" : "no"));
  sprintf(buffer + strlen(buffer), "Has accurate pose: %s\n", (_has_accurate_pose ? "yes" : "no"));
  std::lock_guard<std::mutex> lock1(_mutex_cam);
  cv::Mat pose = getPose();
  if (!pose.empty())
    sprintf(buffer + strlen(buffer), "Pose: Exists [%i x %i]\n", pose.rows, pose.cols);
  std::lock_guard<std::mutex> lock2(_mutex_surface_pts);
  if (!_surface_points.empty())
    sprintf(buffer + strlen(buffer), "Mappoints: %i\n", _surface_points.rows);

  return std::string(buffer);
}

void Frame::computeSceneDepth()
{
  /*
   * Depth computation according to [Hartley2004] "Multiple View Geometry in Computer Vision", S.162:
   * Projection formula P*X = x with P=K(R|t), X=(x,y,z,1) and x=w*(u,v,1)
   * For the last row therefore (p31,p32,p33,p34)*(x,y,z,1)^T=w. If p3=(p31,p32,p33) is the direction of the principal
   * axis and det(p3)>0,||p3||=1 then w can be interpreted as projected depth. Therefore the final formula:
   * w=depth=(r31,r32,r33,t_z)*(x,y,z,1)
   */

  if (_surface_points.empty())
    return;

  std::lock_guard<std::mutex> lock(_mutex_surface_pts);
  int n = _surface_points.rows;
  std::vector<double> depths;
  depths.reserve(n);

  _mutex_cam.lock();
  cv::Mat P = _camera_model->P();
  _mutex_cam.unlock();

  // Prepare extrinsics
  cv::Mat T_w2c = _camera_model->Tw2c();
  cv::Mat R_wc2 = T_w2c.row(2).colRange(0, 3).t();
  double z_wc = T_w2c.at<double>(2, 3);

  for (int i = 0; i < n; ++i)
  {
    cv::Mat pt = _surface_points.row(i).colRange(0, 3).t();

    // Depth calculation
    double depth = R_wc2.dot(pt) + z_wc;
    depths.push_back(depth);
  }
  sort(depths.begin(), depths.end());
  _min_scene_depth = depths[0];
  _max_scene_depth = depths[depths.size() - 1];
  _med_scene_depth = depths[(depths.size() - 1) / 2];
  _is_depth_computed = true;
}

void Frame::updateGeoreference(const cv::Mat &T, bool do_update_surface_points)
{
  // Update the visual pose with the new georeference
  cv::Mat M_c2g = applyTransformationToVisualPose(T);
  setGeographicPose(M_c2g);

  // In case we want to update the surface points as well, we have to compute the difference of old and new transformation.
  if (do_update_surface_points && !_transformation_w2g.empty())
  {
    cv::Mat T_diff = computeGeoreferenceDifference(_transformation_w2g, T);
    applyTransformationToSurfacePoints(T_diff);
  }

  // Pose and / or surface points have changed. So update scene depth accordingly
  computeSceneDepth();

  setGeoreference(T);
}

void Frame::updateOrientation(const cv::Mat &R_u2c)
{
  _orientation = (R_u2c * _orientation.t()).t();
}

void Frame::applyTransformationToSurfacePoints(const cv::Mat &T)
{
  if (_surface_points.rows > 0)
  {
    _mutex_surface_pts.lock();
    for (uint32_t i = 0; i < _surface_points.rows; ++i)
    {
      cv::Mat pt = _surface_points.row(i).colRange(0, 3).t();
      pt.push_back(1.0);
      cv::Mat pt_hom = T * pt;
      pt_hom.pop_back();
      _surface_points.row(i) = pt_hom.t();
    }
    _mutex_surface_pts.unlock();
  }
}

cv::Mat Frame::applyTransformationToVisualPose(const cv::Mat &T)
{
  if (!_motion_c2w.empty())
  {
    cv::Mat T_c2w = cv::Mat::eye(4, 4, CV_64F);
    _motion_c2w.copyTo(T_c2w.rowRange(0, 3).colRange(0, 4));

    cv::Mat T_c2g = T * T_c2w;
    cv::Mat M_c2g = T_c2g.rowRange(0, 3).colRange(0, 4);

    // Remove scale
    M_c2g.col(0) /= cv::norm(M_c2g.col(0));
    M_c2g.col(1) /= cv::norm(M_c2g.col(1));
    M_c2g.col(2) /= cv::norm(M_c2g.col(2));

    return M_c2g;
  }
  return cv::Mat();
}

cv::Mat Frame::computeGeoreferenceDifference(const cv::Mat &T_old, const cv::Mat &T_new)
{
  // Remove scale from old georeference
  cv::Mat T1 = T_old.clone();
  double sx_old = cv::norm(T_old.rowRange(0, 3).col(0));
  double sy_old = cv::norm(T_old.rowRange(0, 3).col(1));
  double sz_old = cv::norm(T_old.rowRange(0, 3).col(2));
  T1.rowRange(0, 3).col(0) /= sx_old;
  T1.rowRange(0, 3).col(1) /= sy_old;
  T1.rowRange(0, 3).col(2) /= sz_old;

  // Remove scale from old georeference
  cv::Mat T2 = T_new.clone();
  double sx_new = cv::norm(T_new.rowRange(0, 3).col(0));
  double sy_new = cv::norm(T_new.rowRange(0, 3).col(1));
  double sz_new = cv::norm(T_new.rowRange(0, 3).col(2));
  T2.rowRange(0, 3).col(0) /= sx_new;
  T2.rowRange(0, 3).col(1) /= sy_new;
  T2.rowRange(0, 3).col(2) /= sz_new;

  cv::Mat T_old_inv = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat R_old_inv = (T1.rowRange(0, 3).colRange(0, 3)).t();
  cv::Mat t_old_inv = -R_old_inv * T1.rowRange(0, 3).col(3);
  R_old_inv.copyTo(T_old_inv.rowRange(0, 3).colRange(0, 3));
  t_old_inv.copyTo(T_old_inv.rowRange(0, 3).col(3));

  cv::Mat T_diff = T_old_inv * T2;

  T_diff.rowRange(0, 3).col(0) *= sx_new / sx_old;
  T_diff.rowRange(0, 3).col(1) *= sy_new / sx_old;
  T_diff.rowRange(0, 3).col(2) *= sz_new / sx_old;

  return T_diff;
}

} // namespace realm