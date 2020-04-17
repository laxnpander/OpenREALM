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

#include <realm_core/frame.h>

namespace realm
{

// CONSTRUCTION
Frame::Frame(const std::string &camera_id,
             const uint32_t &frame_id,
             const uint64_t &timestamp,
             const cv::Mat &img,
             const UTMPose &utm,
             const camera::Pinhole::Ptr &cam)
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
      _cam(cam),
      _img_resize_factor(0.0),
      _min_scene_depth(0.0),
      _max_scene_depth(0.0),
      _med_scene_depth(0.0)
{
  _cam->setPose(getDefaultPose());
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
  return (uint32_t)((double) _cam->width()*_img_resize_factor);
}

uint32_t Frame::getResizedImageHeight() const
{
  assert(_is_img_resizing_set);
  return (uint32_t)((double) _cam->height()*_img_resize_factor);
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
  auto width = (uint32_t)((double) _cam->width()*_img_resize_factor);
  auto height = (uint32_t)((double) _cam->height()*_img_resize_factor);
  return cv::Size(width, height);
}

cv::Mat Frame::getImageUndistorted() const
{
  // - No deep copy
  cv::Mat img_undistorted;
  if(_cam->isDistorted())
    img_undistorted = _cam->undistort(_img, CV_INTER_LINEAR);
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

  // TODO: implement camera mounting
  // Rotation to the world in camera frame
  cv::Mat R_wc = cv::Mat::eye(3, 3, CV_64F);
  R_wc.at<double>(1, 1) = -1;
  R_wc.at<double>(2, 2) = -1;

  // Rotation around z considering uav heading
  double gamma = _utm.heading * M_PI / 180;
  cv::Mat R_wc_z = cv::Mat::eye(3, 3, CV_64F);
  R_wc_z.at<double>(0, 0) = cos(-gamma);
  R_wc_z.at<double>(0, 1) = -sin(-gamma);
  R_wc_z.at<double>(1, 0) = sin(-gamma);
  R_wc_z.at<double>(1, 1) = cos(-gamma);
  cv::Mat R = R_wc_z * R_wc;

  // Create default pose
  cv::Mat default_pose = cv::Mat::eye(3, 4, CV_64F);
  R.copyTo(default_pose.rowRange(0, 3).colRange(0, 3));
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
  camera::Pinhole cam_resized = _cam->resize(_img_resize_factor);
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
  return _cam->resize(_img_resize_factor).K();
}

cv::Mat Frame::getSurfacePoints() const
{
  if (_surface_pts.empty())
    return cv::Mat();
  else
    return _surface_pts.clone();
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
    return _cam->pose();
  }

  // Default:
  return getDefaultPose();
}

cv::Mat Frame::getVisualPose() const
{
  return _M_c2w.clone();
}

cv::Mat Frame::getGeographicPose() const
{
  return _M_c2g.clone();
}

cv::Mat Frame::getGeoreference() const
{
  return _T_w2g.clone();
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

camera::Pinhole::Ptr Frame::getCamera() const
{
  return _cam;
}

uint64_t Frame::getTimestamp() const
{
  // in [nanosec]
  return _timestamp;
}

camera::Pinhole::Ptr Frame::getResizedCamera() const
{
  assert(_is_img_resizing_set);
  return std::make_shared<camera::Pinhole>(_cam->resize(_img_resize_factor));
}

// SETTER

void Frame::setVisualPose(const cv::Mat &pose)
{
  _M_c2w = pose;

  // If frame is already georeferenced, then set camera pose as geographic pose. Otherwise use visual pose
  if (_is_georeferenced)
  {
    updateGeographicPose();
  }
  else
  {
    std::lock_guard<std::mutex> lock(_mutex_cam);
    _cam->setPose(pose);
  }

  setPoseAccurate(true);
}

void Frame::setGeographicPose(const cv::Mat &pose)
{
  std::lock_guard<std::mutex> lock(_mutex_cam);
  _cam->setPose(pose);
  _M_c2g = pose;
  setPoseAccurate(true);
}

void Frame::setGeoreference(const cv::Mat &T_w2g)
{
  if (T_w2g.empty())
    throw(std::invalid_argument("Error setting georeference: Transformation is empty!"));

  std::lock_guard<std::mutex> lock(_mutex_T_w2g);
  _T_w2g = T_w2g;
  _is_georeferenced = true;
}

void Frame::setSurfacePoints(const cv::Mat &surface_pts)
{
  if (surface_pts.empty())
    return;

  _mutex_surface_pts.lock();
  _surface_pts = surface_pts;
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

void Frame::applyGeoreference(const cv::Mat &T)
{
  assert(!T.empty());

  if (_is_georeferenced)
    return;

  setGeoreference(T);
  updateGeographicPose();

  // Also transform mappoints to new coordinate system
  if (_surface_pts.rows > 0)
  {
    _mutex_surface_pts.lock();
    for (uint32_t i = 0; i < _surface_pts.rows; ++i)
    {
      cv::Mat pt = _surface_pts.row(i).colRange(0, 3).t();
      pt.push_back(1.0);
      cv::Mat pt_hom = T * pt;
      pt_hom.pop_back();
      _surface_pts.row(i) = pt_hom.t();
    }
    _mutex_surface_pts.unlock();
    computeSceneDepth();
  }
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
  if (!_surface_pts.empty())
    sprintf(buffer + strlen(buffer), "Mappoints: %i\n", _surface_pts.rows);

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

  if (_surface_pts.empty())
    throw(std::runtime_error("Error computing scene depth: Set point cloud is empty!"));

  std::lock_guard<std::mutex> lock(_mutex_surface_pts);
  int n = _surface_pts.rows;
  std::vector<double> depths;
  depths.reserve(n);

  _mutex_cam.lock();
  cv::Mat P = _cam->P();
  _mutex_cam.unlock();

  // Prepare extrinsics
  cv::Mat T_w2c = _cam->Tw2c();
  cv::Mat R_wc2 = T_w2c.row(2).colRange(0, 3).t();
  double z_wc = T_w2c.at<double>(2, 3);

  for (int i = 0; i < n; ++i)
  {
    cv::Mat pt = _surface_pts.row(i).colRange(0, 3).t();

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

void Frame::updateGeographicPose()
{
  // Only possible, if visual pose AND georeference were computed
  if (_is_georeferenced && !_M_c2w.empty())
  {
    cv::Mat T_c2w = cv::Mat::eye(4, 4, CV_64F);
    _M_c2w.copyTo(T_c2w.rowRange(0, 3).colRange(0, 4));

    cv::Mat T_c2g = _T_w2g * T_c2w;
    cv::Mat M_c2g = T_c2g.rowRange(0, 3).colRange(0, 4);

    // Remove scale
    M_c2g.col(0) /= cv::norm(M_c2g.col(0));
    M_c2g.col(1) /= cv::norm(M_c2g.col(1));
    M_c2g.col(2) /= cv::norm(M_c2g.col(2));

    setGeographicPose(M_c2g);
  }
}

} // namespace realm