

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
    : m_camera_id(camera_id),
      m_frame_id(frame_id),
      m_is_keyframe(false),
      m_is_georeferenced(false),
      m_is_img_resizing_set(false),
      m_is_depth_computed(false),
      m_has_accurate_pose(false),
      m_surface_assumption(SurfaceAssumption::PLANAR),
      m_surface_model(nullptr),
      m_orthophoto(nullptr),
      m_timestamp(timestamp),
      m_img(img),
      m_utm(utm),
      m_camera_model(cam),
      m_orientation(orientation.empty() ? cv::Mat::eye(3, 3, CV_64F) : orientation),
      m_img_resize_factor(0.0),
      m_min_depth(0.0),
      m_max_depth(0.0),
      m_med_depth(0.0),
      m_depthmap(nullptr),
      m_sparse_cloud(nullptr)
{
  if (m_camera_id.empty())
    throw(std::invalid_argument("Error creating frame: Camera Id not provided!"));
  if (!m_camera_model)
    throw(std::invalid_argument("Error creating frame: Camera model not provided!"));
  if (m_img.empty())
    throw(std::invalid_argument("Error creating frame: Image data empty!"));

  m_camera_model->setPose(getDefaultPose());
}

// GETTER

std::string Frame::getCameraId() const
{
  return m_camera_id;
}

uint32_t Frame::getFrameId() const
{
  return m_frame_id;
}

uint32_t Frame::getResizedImageWidth() const
{
  std::lock_guard<std::mutex> lock(m_mutex_cam);
  if (isImageResizeSet())
    return (uint32_t)((double) m_camera_model->width() * m_img_resize_factor);
  else
    throw(std::runtime_error("Error returning resized image width: Image resize factor not set!"));
}

uint32_t Frame::getResizedImageHeight() const
{
  std::lock_guard<std::mutex> lock(m_mutex_cam);
  if (isImageResizeSet())
    return (uint32_t)((double) m_camera_model->height() * m_img_resize_factor);
  else
    throw(std::runtime_error("Error returning resized image height: Image resize factor not set!"));
}

double Frame::getMinSceneDepth() const
{
  if (isDepthComputed())
    return m_min_depth;
  else
    throw(std::runtime_error("Error: Depth was not computed!"));
}

double Frame::getMaxSceneDepth() const
{
  if (isDepthComputed())
    return m_max_depth;
  else
    throw(std::runtime_error("Error: Depth was not computed!"));
}

double Frame::getMedianSceneDepth() const
{
  if (isDepthComputed())
    return m_med_depth;
  else
    throw(std::runtime_error("Error: Depth was not computed!"));
}

Depthmap::Ptr Frame::getDepthmap() const
{
  return m_depthmap;
}

cv::Size Frame::getResizedImageSize() const
{
  if (isImageResizeSet())
  {
    auto width = (uint32_t)((double) m_camera_model->width() * m_img_resize_factor);
    auto height = (uint32_t)((double) m_camera_model->height() * m_img_resize_factor);
    return cv::Size(width, height);
  }
  else
    throw(std::runtime_error("Error: Image resize factor not set!"));
}

cv::Mat Frame::getImageUndistorted() const
{
  std::lock_guard<std::mutex> lock(m_mutex_cam);
  cv::Mat img_undistorted;

  if(m_camera_model->hasDistortion())
    img_undistorted = m_camera_model->undistort(m_img, cv::InterpolationFlags::INTER_LINEAR);
  else
    img_undistorted = m_img;
  return std::move(img_undistorted);
}

cv::Mat Frame::getImageRaw() const
{
  // - No deep copy
  return m_img;
}

cv::Mat Frame::getDefaultPose() const
{
  // Translation set to measured utm coordinates
  cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
  t.at<double>(0) = m_utm.easting;
  t.at<double>(1) = m_utm.northing;
  t.at<double>(2) = m_utm.altitude;

  // Create default pose
  cv::Mat default_pose = cv::Mat::eye(3, 4, CV_64F);
  m_orientation.copyTo(default_pose.rowRange(0, 3).colRange(0, 3));
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
  if (isImageResizeSet())
  {
    camera::Pinhole cam_resized = m_camera_model->resize(m_img_resize_factor);
    return cam_resized.undistort(m_img_resized, cv::InterpolationFlags::INTER_LINEAR);
  }
  else
    throw(std::invalid_argument("Error: Image resize factor not set!"));
}

cv::Mat Frame::getResizedImageRaw() const
{
  // deep copy, as it might be painted or modified
  return m_img_resized.clone();
}

cv::Mat Frame::getResizedCalibration() const
{
  std::lock_guard<std::mutex> lock(m_mutex_cam);
  if (isImageResizeSet())
    return m_camera_model->resize(m_img_resize_factor).K();
  else
    throw(std::runtime_error("Error resizing camera: Image resizing was not set!"));
}

PointCloud::Ptr Frame::getSparseCloud() const
{
  std::lock_guard<std::mutex> lock(m_mutex_sparse_points);
  return m_sparse_cloud;
}

void Frame::setDepthmap(const Depthmap::Ptr &depthmap)
{
  m_depthmap = depthmap;
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
    return m_camera_model->pose();
  }

  // Default:
  return getDefaultPose();
}

cv::Mat Frame::getVisualPose() const
{
  return m_motion_c2w.clone();
}

cv::Mat Frame::getGeographicPose() const
{
  return m_motion_c2g.clone();
}

cv::Mat Frame::getGeoreference() const
{
  return m_transformation_w2g.clone();
}

SurfaceAssumption Frame::getSurfaceAssumption() const
{
  std::lock_guard<std::mutex> lock(m_mutex_flags);
  return m_surface_assumption;
}

CvGridMap::Ptr Frame::getSurfaceModel() const
{
  std::lock_guard<std::mutex> lock(m_mutex_surface_model);
  return m_surface_model;
}

CvGridMap::Ptr Frame::getOrthophoto() const
{
  std::lock_guard<std::mutex> lock(m_mutex_orthophoto);
  return m_orthophoto;
}

UTMPose Frame::getGnssUtm() const
{
  return m_utm;
}

camera::Pinhole::ConstPtr Frame::getCamera() const
{
  std::lock_guard<std::mutex> lock(m_mutex_cam);
  return m_camera_model;
}

uint64_t Frame::getTimestamp() const
{
  // in [nanosec]
  return m_timestamp;
}

camera::Pinhole::Ptr Frame::getResizedCamera() const
{
  assert(m_is_img_resizing_set);
  return std::make_shared<camera::Pinhole>(m_camera_model->resize(m_img_resize_factor));
}

// SETTER

void Frame::setVisualPose(const cv::Mat &pose)
{
  m_motion_c2w = pose;

  // If frame is already georeferenced, then set camera pose as geographic pose. Otherwise use visual pose
  if (m_is_georeferenced)
  {
    cv::Mat M_c2g = applyTransformationToVisualPose(m_transformation_w2g);
    setGeographicPose(M_c2g);
  }
  else
  {
    std::lock_guard<std::mutex> lock(m_mutex_cam);
    m_camera_model->setPose(pose);
  }

  setPoseAccurate(true);
}

void Frame::setGeographicPose(const cv::Mat &pose)
{
  if (!pose.empty())
  {
    std::lock_guard<std::mutex> lock(m_mutex_cam);
    m_camera_model->setPose(pose);
    m_motion_c2g = pose;
    setPoseAccurate(true);
  }
}

void Frame::setGeoreference(const cv::Mat &T_w2g)
{
  if (T_w2g.empty())
    throw(std::invalid_argument("Error setting georeference: Transformation is empty!"));

  std::lock_guard<std::mutex> lock(m_mutex_T_w2g);
  m_transformation_w2g = T_w2g.clone();
  m_is_georeferenced = true;
}

void Frame::setSparseCloud(const PointCloud::Ptr &sparse_cloud, bool in_visual_coordinates)
{
  if (sparse_cloud->empty())
    return;

  m_mutex_sparse_points.lock();
  m_sparse_cloud = sparse_cloud;
  m_mutex_sparse_points.unlock();

  m_mutex_flags.lock();
  if (in_visual_coordinates && m_is_georeferenced)
  {
    m_mutex_T_w2g.lock();
    applyTransformationToSparseCloud(m_transformation_w2g);
    m_mutex_T_w2g.unlock();
  }
  m_mutex_flags.unlock();

  computeSceneDepth(1000);
}

void Frame::setSurfaceModel(const CvGridMap::Ptr &surface_model)
{
  std::lock_guard<std::mutex> lock(m_mutex_surface_model);
  m_surface_model = surface_model;
}

void Frame::setOrthophoto(const CvGridMap::Ptr &orthophoto)
{
  std::lock_guard<std::mutex> lock(m_mutex_orthophoto);
  m_orthophoto = orthophoto;
}

void Frame::setKeyframe(bool flag)
{
  std::lock_guard<std::mutex> lock(m_mutex_flags);
  m_is_keyframe = flag;
}

void Frame::setPoseAccurate(bool flag)
{
  std::lock_guard<std::mutex> lock(m_mutex_flags);
  // If we are clearing the flag, clear the georeferenced pose data as well
  if (!flag) {
    m_camera_model->setPose(getDefaultPose());
  }
  m_has_accurate_pose = flag;
}

void Frame::setSurfaceAssumption(SurfaceAssumption assumption)
{
  std::lock_guard<std::mutex> lock(m_mutex_flags);
  m_surface_assumption = assumption;
}

void Frame::setImageResizeFactor(const double &value)
{
  std::lock_guard<std::mutex> lock(m_mutex_img_resized);
  m_img_resize_factor = value;
  cv::resize(m_img, m_img_resized, cv::Size(), m_img_resize_factor, m_img_resize_factor);
  m_is_img_resizing_set = true;
}


// FUNCTIONALITY

void Frame::initGeoreference(const cv::Mat &T)
{
  assert(!T.empty());

  m_transformation_w2g = cv::Mat::eye(4, 4, CV_64F);
  updateGeoreference(T, true);
}

bool Frame::isKeyframe() const
{
  std::lock_guard<std::mutex> lock(m_mutex_flags);
  return m_is_keyframe;
}

bool Frame::isGeoreferenced() const
{
  std::lock_guard<std::mutex> lock(m_mutex_flags);
  return m_is_georeferenced;
}

bool Frame::isImageResizeSet() const
{
  std::lock_guard<std::mutex> lock(m_mutex_flags);
  return m_is_img_resizing_set;
}

bool Frame::isDepthComputed() const
{
  std::lock_guard<std::mutex> lock(m_mutex_flags);
  return m_is_depth_computed;
}

bool Frame::hasAccuratePose() const
{
  std::lock_guard<std::mutex> lock(m_mutex_flags);
  return m_has_accurate_pose;
}

std::string Frame::print()
{
  std::lock_guard<std::mutex> lock(m_mutex_flags);
  char buffer[5000];
  sprintf(buffer, "### FRAME INFO ###\n");
  sprintf(buffer + strlen(buffer), "Stamp: %lu \n", m_timestamp);
  sprintf(buffer + strlen(buffer), "Image: [%i x %i]\n", m_img.cols, m_img.rows);
  sprintf(buffer + strlen(buffer), "GNSS: [%4.02f E, %4.02f N, %4.02f Alt, %4.02f Head]\n",
          m_utm.easting, m_utm.northing, m_utm.altitude, m_utm.heading);
  sprintf(buffer + strlen(buffer), "Is key frame: %s\n", (m_is_keyframe ? "yes" : "no"));
  sprintf(buffer + strlen(buffer), "Has accurate pose: %s\n", (m_has_accurate_pose ? "yes" : "no"));
  std::lock_guard<std::mutex> lock1(m_mutex_cam);
  cv::Mat pose = getPose();
  if (!pose.empty())
    sprintf(buffer + strlen(buffer), "Pose: Exists [%i x %i]\n", pose.rows, pose.cols);
  std::lock_guard<std::mutex> lock2(m_mutex_sparse_points);
  if (!m_sparse_cloud->empty())
    sprintf(buffer + strlen(buffer), "Mappoints: %i\n", m_sparse_cloud->data().rows);

  return std::string(buffer);
}

// TODO: move this into sparse cloud?
void Frame::computeSceneDepth(int max_nrof_points)
{
  /*
   * Depth computation according to [Hartley2004] "Multiple View Geometry in Computer Vision", S.162:
   * Projection formula P*X = x with P=K(R|t), X=(x,y,z,1) and x=w*(u,v,1)
   * For the last row therefore (p31,p32,p33,p34)*(x,y,z,1)^T=w. If p3=(p31,p32,p33) is the direction of the principal
   * axis and det(p3)>0,||p3||=1 then w can be interpreted as projected depth. Therefore the final formula:
   * w=depth=(r31,r32,r33,t_z)*(x,y,z,1)
   */

  if (!m_sparse_cloud || m_sparse_cloud->empty())
    return;

  std::lock_guard<std::mutex> lock(m_mutex_sparse_points);

  // The user can limit the number of points on which the depth is computed. The increment for the iteration later on
  // is changed accordingly.
  int n = 0;
  int inc = 1;

  cv::Mat sparse_data = m_sparse_cloud->data();
  if (max_nrof_points == 0 || sparse_data.rows < max_nrof_points)
  {
    n = sparse_data.rows;
  }
  else
  {
    n = max_nrof_points;
    inc = sparse_data.rows * (max_nrof_points / sparse_data.rows);

    // Just to make sure we don't run into an infinite loop
    if (inc <= 0)
      inc = 1;
  }

  std::vector<double> depths;
  depths.reserve(n);

  m_mutex_cam.lock();
  cv::Mat P = m_camera_model->P();
  m_mutex_cam.unlock();

  // Prepare extrinsics
  cv::Mat T_w2c = m_camera_model->Tw2c();
  cv::Mat R_wc2 = T_w2c.row(2).colRange(0, 3).t();
  double z_wc = T_w2c.at<double>(2, 3);

  for (int i = 0; i < n; i += inc)
  {
    cv::Mat pt = sparse_data.row(i).colRange(0, 3).t();

    // Depth calculation
    double depth = R_wc2.dot(pt) + z_wc;
    depths.push_back(depth);
  }
  sort(depths.begin(), depths.end());
  m_min_depth = depths[0];
  m_max_depth = depths[depths.size() - 1];
  m_med_depth = depths[(depths.size() - 1) / 2];
  m_is_depth_computed = true;
}

void Frame::updateGeoreference(const cv::Mat &T, bool do_update_sparse_cloud)
{
  // Update the visual pose with the new georeference
  cv::Mat M_c2g = applyTransformationToVisualPose(T);
  setGeographicPose(M_c2g);

  // In case we want to update the surface points as well, we have to compute the difference of old and new transformation.
  if (do_update_sparse_cloud && !m_transformation_w2g.empty())
  {
    cv::Mat T_diff = computeGeoreferenceDifference(m_transformation_w2g, T);
    applyTransformationToSparseCloud(T_diff);
  }

  // Pose and / or surface points have changed. So update scene depth accordingly
  computeSceneDepth(1000);

  setGeoreference(T);
}

cv::Mat Frame::getOrientation() const
{
  return m_orientation.clone();
}

void Frame::applyTransformationToSparseCloud(const cv::Mat &T)
{
  if (m_sparse_cloud && !m_sparse_cloud->empty())
  {
    cv::Mat sparse_data = m_sparse_cloud->data();
    m_mutex_sparse_points.lock();
    for (uint32_t i = 0; i < sparse_data.rows; ++i)
    {
      cv::Mat pt = sparse_data.row(i).colRange(0, 3).t();
      pt.push_back(1.0);
      cv::Mat pt_hom = T * pt;
      pt_hom.pop_back();
      sparse_data.row(i) = pt_hom.t();
    }
    m_mutex_sparse_points.unlock();
  }
}

cv::Mat Frame::applyTransformationToVisualPose(const cv::Mat &T)
{
  if (!m_motion_c2w.empty())
  {
    cv::Mat T_c2w = cv::Mat::eye(4, 4, CV_64F);
    m_motion_c2w.copyTo(T_c2w.rowRange(0, 3).colRange(0, 4));

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