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

#include <realm_ros/conversions.h>

namespace realm
{

cv_bridge::CvImage to_ros::image(const std_msgs::Header &header, const cv::Mat &cv_img)
{
  std::string encoding;
  switch (cv_img.type())
  {
    case CV_32FC1:
      encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      break;
    case CV_32FC3:
      encoding = sensor_msgs::image_encodings::TYPE_32FC3;
      break;
    case CV_64F:
      encoding = sensor_msgs::image_encodings::TYPE_64FC1;
      break;
    case CV_16UC1:
      encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      break;
    case CV_8UC1:
      encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      break;
    case CV_8UC3:
      encoding = sensor_msgs::image_encodings::TYPE_8UC3;
      break;
    case CV_8UC4:
      encoding = sensor_msgs::image_encodings::TYPE_8UC4;
      break;
    default:
      throw std::out_of_range("Error convertig OpenCV Mat: Unknown data type.");
  }
  return cv_bridge::CvImage(header, encoding, cv_img);
}

cv_bridge::CvImage to_ros::imageDisplay(const std_msgs::Header &header, const cv::Mat &cv_img)
{
  std::string encoding;
  switch (cv_img.type())
  {
    case CV_32F:
      encoding = sensor_msgs::image_encodings::MONO16;
      break;
    case CV_64F:
      encoding = sensor_msgs::image_encodings::MONO16;
      break;
    case CV_16UC1:
      encoding = sensor_msgs::image_encodings::MONO16;
      break;
    case CV_8UC1:
      encoding = sensor_msgs::image_encodings::MONO8;
      break;
    case CV_8UC3:
      encoding = sensor_msgs::image_encodings::BGR8;
      break;
    case CV_8UC4:
      encoding = sensor_msgs::image_encodings::BGRA8;
      break;
    default:
      throw std::out_of_range("Error converting image to ROS message: Unknown mat type.");
  }
  return cv_bridge::CvImage(header, encoding, cv_img);
}

geodesy::UTMPoint to_ros::utm(const realm::UTMPose &r_utm)
{
  geodesy::UTMPoint g_utm;
  g_utm.easting = r_utm.easting;
  g_utm.northing = r_utm.northing;
  g_utm.altitude = r_utm.altitude;
  g_utm.zone = r_utm.zone;
  g_utm.band = r_utm.band;
  return g_utm;
}

geographic_msgs::GeoPoint to_ros::wgs84(const realm::UTMPose &r_utm)
{
  geodesy::UTMPoint g_utm = to_ros::utm(r_utm);
  return geodesy::toMsg(g_utm);
}


realm::UTMPose to_realm::utm(const sensor_msgs::NavSatFix &gnss, const std_msgs::Float32 &heading)
{
  geographic_msgs::GeoPoint wgs;
  wgs.latitude = gnss.latitude;
  wgs.longitude = gnss.longitude;
  wgs.altitude = gnss.altitude;
  geodesy::UTMPoint g_utm(wgs);

  realm::UTMPose r_utm;
  r_utm.northing = g_utm.northing;
  r_utm.easting = g_utm.easting;
  r_utm.altitude = g_utm.altitude;
  r_utm.zone = g_utm.zone;
  r_utm.band = g_utm.band;
  r_utm.heading = (double)heading.data;

  return r_utm;
}

tf::Transform to_ros::tf(const geometry_msgs::Pose &msg)
{
  tf::Vector3 origin(msg.position.x, msg.position.y, msg.position.z);
  tf::Quaternion quat(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  return tf::Transform(quat, origin);
}

tf::Transform to_ros::tf(const cv::Mat &cv_pose)
{
  geometry_msgs::Pose msg_pose = to_ros::pose(cv_pose);
  return to_ros::tf(msg_pose);
}

geometry_msgs::Transform to_ros::tfMsg(const cv::Mat &T)
{
  tf::Transform transform = to_ros::tf(T);

  geometry_msgs::Transform msg;
  msg.translation.x = transform.getOrigin().x();
  msg.translation.y = transform.getOrigin().y();
  msg.translation.z = transform.getOrigin().z();
  msg.rotation.x = transform.getRotation().x();
  msg.rotation.y = transform.getRotation().y();
  msg.rotation.z = transform.getRotation().z();
  msg.rotation.w = transform.getRotation().w();

  return msg;
}

realm_msgs::Georeference to_ros::georeference(const cv::Mat &mat)
{
  realm_msgs::Georeference msg;

  cv::Mat m = mat.rowRange(0, 3).colRange(0, 4);

  double sx = cv::norm(m.col(0));
  double sy = cv::norm(m.col(1));
  double sz = cv::norm(m.col(2));

  msg.scale.resize(3);
  msg.scale[0].data = sx;
  msg.scale[1].data = sy;
  msg.scale[2].data = sz;
  m.col(0) /= sx;
  m.col(1) /= sy;
  m.col(2) /= sz;

  tf::Vector3 tf_pos(mat.at<double>(0, 3), mat.at<double>(1, 3), mat.at<double>(2, 3));
  tf::Matrix3x3 tf_rot(mat.at<double>(0, 0), mat.at<double>(0, 1), mat.at<double>(0, 2),
                       mat.at<double>(1, 0), mat.at<double>(1, 1), mat.at<double>(1, 2),
                       mat.at<double>(2, 0), mat.at<double>(2, 1), mat.at<double>(2, 2));
  tf::Transform tf_pose(tf_rot, tf_pos);

  msg.transform.translation.x = tf_pose.getOrigin().getX();
  msg.transform.translation.y = tf_pose.getOrigin().getY();
  msg.transform.translation.z = tf_pose.getOrigin().getZ();
  msg.transform.rotation.x = tf_pose.getRotation().getX();
  msg.transform.rotation.y = tf_pose.getRotation().getY();
  msg.transform.rotation.z = tf_pose.getRotation().getZ();
  msg.transform.rotation.w = tf_pose.getRotation().getW();
  return msg;
}

tf::StampedTransform to_ros::tfStamped(const geometry_msgs::PoseStamped &msg)
{
  tf::Transform transform = to_ros::tf(msg.pose);
  tf::StampedTransform stamped_transform;
  stamped_transform.stamp_ = msg.header.stamp;
  stamped_transform.frame_id_ = msg.header.frame_id;
  stamped_transform.setOrigin(transform.getOrigin());
  stamped_transform.setRotation(transform.getRotation());
  return stamped_transform;
}

realm::Frame::Ptr to_realm::frame(const realm_msgs::Frame &msg)
{
  // Essential sensor data
  cv::Mat img = to_realm::imageCompressed(msg.imagedata);
  realm::UTMPose utm = to_realm::utm(msg.gpsdata, msg.heading);
  realm::camera::Pinhole::Ptr cam = to_realm::pinhole(msg.camera_model);
  realm::Frame::Ptr frame = std::make_shared<realm::Frame>(msg.camera_id.data, msg.stage_id.data, msg.timestamp.data, img, utm, cam);

  // Optional generated data
  if (msg.has_accurate_pose.data)
    frame->setVisualPose(to_realm::pose(msg.visual_pose));
  if (msg.is_georeferenced.data)
    frame->updateGeoreference(to_realm::georeference(msg.georeference));
  if (msg.observed_map.length_x > 0 && msg.observed_map.length_y > 0)
    frame->setObservedMap(to_realm::cvGridMap(msg.observed_map));
  if (msg.is_keyframe.data)
    frame->setKeyframe(true);
  if (msg.is_surface_elevated.data)
    frame->setSurfaceAssumption(realm::SurfaceAssumption::ELEVATION);

  cv::Mat pcl = to_realm::pointCloud(msg.surface_points);
  if (pcl.cols >= 3 && pcl.rows > 5)
    frame->setSurfacePoints(pcl);

  return std::move(frame);
}


realm_msgs::Pinhole to_ros::pinhole(const realm::camera::Pinhole::ConstPtr &cam)
{
  realm_msgs::Pinhole msg;
  msg.width.data = cam->width();
  msg.height.data = cam->height();
  msg.fx.data = cam->fx();
  msg.fy.data = cam->fy();
  msg.cx.data = cam->cx();
  msg.cy.data = cam->cy();
  msg.k1.data = cam->k1();
  msg.k2.data = cam->k2();
  msg.p1.data = cam->p1();
  msg.p2.data = cam->p2();
  msg.k3.data = cam->k3();
  return msg;
}

realm::camera::Pinhole::Ptr to_realm::pinhole(const realm_msgs::Pinhole &msg)
{
  realm::camera::Pinhole cam(msg.fx.data, msg.fy.data, msg.cx.data, msg.cy.data, msg.width.data, msg.height.data);
  cam.setDistortionMap(msg.k1.data, msg.k2.data, msg.p1.data, msg.p2.data, msg.k3.data);
  return std::make_shared<camera::Pinhole>(cam);
}

sensor_msgs::PointCloud2 to_ros::pointCloud(const std_msgs::Header &header, const cv::Mat &points)
{
  assert(points.cols >= 3);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  for (uint32_t i = 0; i < points.rows; ++i)
  {
    pcl::PointXYZRGBNormal pt;

    // Position informations are saved in 0,1,2
    pt.x = (float)points.at<double>(i, 0);
    pt.y = (float)points.at<double>(i, 1);
    pt.z = (float)points.at<double>(i, 2);

    // Color informations are saved in 3,4,5
    if (points.cols > 3)
    {
      auto r = static_cast<uint32_t>(points.at<double>(i, 3)*255.0);
      auto g = static_cast<uint32_t>(points.at<double>(i, 4)*255.0);
      auto b = static_cast<uint32_t>(points.at<double>(i, 5)*255.0);
      uint32_t rgb = (r << 16 | g << 8 | b);
      pt.rgb = *reinterpret_cast<float*>(&rgb);
    }

    // Point normal informations are saved in 6,7,8
    if (points.cols > 6)
    {
      pt.normal_x = (float)points.at<double>(i, 6);
      pt.normal_y = (float)points.at<double>(i, 7);
      pt.normal_z = (float)points.at<double>(i, 8);
    }

    pcl_ptr->points.push_back(pt);
  }

  pcl_ptr->width = (int) pcl_ptr->points.size();
  pcl_ptr->height = 1;
  sensor_msgs::PointCloud2 pcl_msg;
  pcl::toROSMsg(*pcl_ptr, pcl_msg);
  pcl_msg.header = header;
  return pcl_msg;
}

geometry_msgs::Pose to_ros::pose(const cv::Mat &cv_pose)
{
  tf::Vector3 tf_pos_vec(cv_pose.at<double>(0, 3), cv_pose.at<double>(1, 3), cv_pose.at<double>(2, 3));

  tf::Matrix3x3 tf_rot_mat(cv_pose.at<double>(0, 0), cv_pose.at<double>(0, 1), cv_pose.at<double>(0, 2),
                           cv_pose.at<double>(1, 0), cv_pose.at<double>(1, 1), cv_pose.at<double>(1, 2),
                           cv_pose.at<double>(2, 0), cv_pose.at<double>(2, 1), cv_pose.at<double>(2, 2));

  tf::Transform tf_pose(tf_rot_mat, tf_pos_vec);

  geometry_msgs::Pose msg_pose;
  msg_pose.position.x = tf_pos_vec.getX();
  msg_pose.position.y = tf_pos_vec.getY();
  msg_pose.position.z = tf_pos_vec.getZ();
  msg_pose.orientation.x = tf_pose.getRotation().getX();
  msg_pose.orientation.y = tf_pose.getRotation().getY();
  msg_pose.orientation.z = tf_pose.getRotation().getZ();
  msg_pose.orientation.w = tf_pose.getRotation().getW();

  return msg_pose;
}

geometry_msgs::Pose to_ros::pose(const tf::Transform &transform)
{
  geometry_msgs::Pose pose;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();
  pose.orientation.x = transform.getRotation().x();
  pose.orientation.y = transform.getRotation().y();
  pose.orientation.z = transform.getRotation().z();
  pose.orientation.w = transform.getRotation().w();
  return pose;
}

cv::Mat to_realm::pose(const tf::Transform &transform)
{
  geometry_msgs::Pose ros_pose = to_ros::pose(transform);
  return to_realm::pose(ros_pose);
}

cv::Mat to_realm::tf(const geometry_msgs::Transform &msg)
{
  tf::Vector3 p(msg.translation.x, msg.translation.y, msg.translation.z);
  tf::Quaternion quat(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w);
  tf::Transform tf(quat, p);
  return to_realm::pose(tf);
}

cv::Mat to_realm::georeference(const realm_msgs::Georeference &msg)
{
  tf::Vector3 t(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z);
  tf::Matrix3x3 rot(tf::Quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w));

  cv::Mat T_euclidean = (cv::Mat_<double>(4, 4, CV_64F)
      << rot[0].getX(), rot[0].getY(), rot[0].getZ(), t.getX(),
         rot[1].getX(), rot[1].getY(), rot[1].getZ(), t.getY(),
         rot[2].getX(), rot[2].getY(), rot[2].getZ(), t.getZ(),
         0.0, 0.0, 0.0, 1.0);

  T_euclidean.col(0)*=msg.scale[0].data;
  T_euclidean.col(1)*=msg.scale[1].data;
  T_euclidean.col(2)*=msg.scale[2].data;

  return T_euclidean;
}

geometry_msgs::Pose to_ros::poseWgs84(const cv::Mat &cv_pose, uint8_t zone, char band)
{
  geometry_msgs::Pose utm_pose = to_ros::pose(cv_pose);
  geodesy::UTMPoint utm(utm_pose.position.x, utm_pose.position.y, utm_pose.position.z, zone, band);
  geographic_msgs::GeoPoint wgs = geodesy::toMsg(utm);
  geometry_msgs::Pose wgs_pose;
  wgs_pose.position.x = wgs.longitude;
  wgs_pose.position.y = wgs.latitude;
  wgs_pose.position.z = wgs.altitude;
  wgs_pose.orientation = utm_pose.orientation;
  return wgs_pose;
}


cv::Mat to_realm::pose(const geometry_msgs::Pose &ros_pose)
{
  tf::Vector3 p(ros_pose.position.x, ros_pose.position.y, ros_pose.position.z);
  tf::Quaternion quat(ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w);
  tf::Transform tf(quat, p);

  tf::Vector3 t = tf.getOrigin();
  tf::Matrix3x3 rot = tf.getBasis();
  cv::Mat cv_pose = (cv::Mat_<double>(3, 4, CV_64F) <<
      rot[0].getX(), rot[0].getY(), rot[0].getZ(), p.getX(),
      rot[1].getX(), rot[1].getY(), rot[1].getZ(), p.getY(),
      rot[2].getX(), rot[2].getY(), rot[2].getZ(), p.getZ());
  return cv_pose;
}

realm_msgs::Frame to_ros::frame(const std_msgs::Header &header, const realm::Frame::Ptr &frame)
{
  realm_msgs::Frame msg;

  msg.header = header;
  msg.imagedata = *to_ros::imageDisplay(header, frame->getImageRaw()).toCompressedImageMsg(cv_bridge::JPG);
  msg.camera_id.data = frame->getCameraId();
  msg.stage_id.data = frame->getFrameId();
  msg.timestamp.data = frame->getTimestamp();
  msg.camera_model = to_ros::pinhole(frame->getCamera());

  geodesy::UTMPoint geo_utm;
  geo_utm.easting = frame->getGnssUtm().easting;
  geo_utm.northing = frame->getGnssUtm().northing;
  geo_utm.altitude = frame->getGnssUtm().altitude;
  geo_utm.zone = frame->getGnssUtm().zone;
  geo_utm.band = frame->getGnssUtm().band;
  geographic_msgs::GeoPoint geo_wgs = geodesy::toMsg(geo_utm);

  msg.gpsdata.header = header;
  msg.gpsdata.latitude = geo_wgs.latitude;
  msg.gpsdata.longitude = geo_wgs.longitude;
  msg.gpsdata.altitude = geo_wgs.altitude;
  msg.heading.data = static_cast<float>(frame->getGnssUtm().heading);

  if (frame->hasAccuratePose())
  {
    // Important: Pose in the message is the visual pose, not the combined solution from frame
    msg.visual_pose = to_ros::pose(frame->getVisualPose());
    msg.has_accurate_pose.data = 1;
  }
  if (frame->isGeoreferenced())
  {
    msg.georeference = to_ros::georeference(frame->getGeoreference());
    msg.is_georeferenced.data = 1;
  }

  cv::Mat map_points = frame->getSurfacePoints();
  if (map_points.rows > 5)
  {
    cv_bridge::CvImage img = to_ros::image(header, frame->getSurfacePoints());
    msg.surface_points = *img.toImageMsg();
  }
  if (frame->hasObservedMap())
    msg.observed_map = to_ros::cvGridMap(header, frame->getObservedMap());
  if (frame->isKeyframe())
    msg.is_keyframe.data = 1;
  if (frame->getSurfaceAssumption() == realm::SurfaceAssumption::ELEVATION)
    msg.is_surface_elevated.data = 1;

  return msg;
}

realm_msgs::GroundImageCompressed to_ros::groundImage(const std_msgs::Header &header,
                                                      const cv::Mat &img,
                                                      const realm::UTMPose &ulc,
                                                      double GSD,
                                                      const cv::Mat &mask)
{
  // TODO: Mask for BGR/A images necessary? -> Overall very specialised function currently

  realm_msgs::GroundImageCompressed msg;
  if (img.type() == CV_8UC1 || img.type() == CV_8UC3)
    msg.imagedata = *cv_bridge::CvImage(header, "rgb8", img).toCompressedImageMsg(cv_bridge::PNG);
  else if (img.type() == CV_8UC4)
    msg.imagedata = *cv_bridge::CvImage(header, "rgba8", img).toCompressedImageMsg(cv_bridge::PNG);
  else
    throw(std::invalid_argument("Error converting ground image: Floating point currently not supported!"));

  geographic_msgs::GeoPoint wgs = to_ros::wgs84(ulc);
  msg.gpsdata.latitude = wgs.latitude;
  msg.gpsdata.longitude = wgs.longitude;
  msg.gpsdata.altitude = wgs.altitude;
  msg.scale = GSD;
  return msg;
}

std::vector<Face> to_ros::fixRvizMeshFlickerBug(const std::vector<realm::Face> &faces, const tf::Transform &T)
{
  cv::Mat pose = to_realm::pose(T);
  cv::Mat hom = (cv::Mat_<double>(1, 4) << 0.0, 0.0, 0.0, 1.0);
  pose.push_back(hom);
  std::vector<Face> faces_fixed(faces.size());
  for (size_t i = 0; i < faces.size(); ++i)
    for (size_t j = 0; j < 3; ++j)
    {
      cv::Mat pt = (cv::Mat_<double>(4, 1) << faces[i].vertices[j].x, faces[i].vertices[j].y, faces[i].vertices[j].z, 1.0);
      cv::Mat pt_t = pose*pt;
      faces_fixed[i].vertices[j].x = pt_t.at<double>(0)/pt_t.at<double>(3);
      faces_fixed[i].vertices[j].y = pt_t.at<double>(1)/pt_t.at<double>(3);
      faces_fixed[i].vertices[j].z = pt_t.at<double>(2)/pt_t.at<double>(3);
      faces_fixed[i].color[j] = faces[i].color[j];
    }
  return faces_fixed;
}

visualization_msgs::Marker to_ros::meshMarker(const std_msgs::Header &header,
                                              const std::vector<Face> &faces,
                                              const std::string &ns,
                                              int32_t id,
                                              int32_t type,
                                              int32_t action,
                                              const tf::Transform &T)
{
  std::vector<Face> faces_fixed = fixRvizMeshFlickerBug(faces, T);

  visualization_msgs::Marker msg;
  msg.header = header;
  msg.ns = "Global Map";
  msg.id = id;
  msg.type = type;
  msg.action = action;
  msg.pose.position.x = 0;
  msg.pose.position.y = 0;
  msg.pose.position.z = 0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = 1;
  msg.scale.y = 1;
  msg.scale.z = 1;
  msg.color.a = 1.0; // Don't forget to set the alpha!
  msg.color.r = 0.0;
  msg.color.g = 1.0;
  msg.color.b = 0.0;
  msg.points.reserve(faces_fixed.size()*3);
  msg.colors.reserve(faces_fixed.size()*3);

  for (const auto &face : faces_fixed)
  {
    if (cv::norm(face.vertices[0] - face.vertices[1]) > 5.0)
      continue;
    if (cv::norm(face.vertices[1] - face.vertices[2]) > 5.0)
      continue;

    geometry_msgs::Point pt1;
    pt1.x = face.vertices[0].x;
    pt1.y = face.vertices[0].y;
    pt1.z = face.vertices[0].z;

    std_msgs::ColorRGBA rgba1;
    rgba1.b = static_cast<float>(face.color[0][0])/255.0f;
    rgba1.g = static_cast<float>(face.color[0][1])/255.0f;
    rgba1.r = static_cast<float>(face.color[0][2])/255.0f;
    rgba1.a = 1.0;

    geometry_msgs::Point pt2;
    pt2.x = face.vertices[1].x;
    pt2.y = face.vertices[1].y;
    pt2.z = face.vertices[1].z;

    std_msgs::ColorRGBA rgba2;
    rgba2.b = static_cast<float>(face.color[1][0])/255.0f;
    rgba2.g = static_cast<float>(face.color[1][1])/255.0f;
    rgba2.r = static_cast<float>(face.color[1][2])/255.0f;
    rgba2.a = 1.0;

    geometry_msgs::Point pt3;
    pt3.x = face.vertices[2].x;
    pt3.y = face.vertices[2].y;
    pt3.z = face.vertices[2].z;

    std_msgs::ColorRGBA rgba3;
    rgba3.b = static_cast<float>(face.color[2][0])/255.0f;
    rgba3.g = static_cast<float>(face.color[2][1])/255.0f;
    rgba3.r = static_cast<float>(face.color[2][2])/255.0f;
    rgba3.a = 1.0;

    msg.points.push_back(pt1);
    msg.points.push_back(pt2);
    msg.points.push_back(pt3);
    msg.colors.push_back(rgba1);
    msg.colors.push_back(rgba2);
    msg.colors.push_back(rgba3);
  }
  return msg;
}

realm_msgs::CvGridMap to_ros::cvGridMap(const std_msgs::Header &header, const realm::CvGridMap::Ptr &map)
{
  realm_msgs::CvGridMap msg;
  msg.header = header;
  msg.resolution = map->resolution();

  cv::Rect2d roi = map->roi();
  geometry_msgs::Point pos;
  pos.x = roi.x;
  pos.y =  roi.y;
  pos.z = 0.0;
  msg.pos = pos;
  msg.length_x = roi.width;
  msg.length_y = roi.height;

  std::vector<std::string> layer_names = map->getAllLayerNames();
  std::vector<sensor_msgs::Image> layer_data;
  for (const auto &layer_name : layer_names)
  {
    cv_bridge::CvImage img = to_ros::image(header, (*map)[layer_name]);
    layer_data.push_back(*img.toImageMsg());
  }
  msg.layers = layer_names;
  msg.data = layer_data;
  return msg;
}

cv::Mat realm::to_realm::pointCloud(const sensor_msgs::Image &msg)
{
  cv::Mat point_cloud;
  try
  {
    point_cloud = (*cv_bridge::toCvCopy(msg)).image;
  }
  catch(...)
  {

  }
  return point_cloud;
}

cv::Mat realm::to_realm::image(const sensor_msgs::Image &msg)
{
  cv_bridge::CvImagePtr img_ptr;
  try
  {
    img_ptr = cv_bridge::toCvCopy(msg);
  }
  catch(...)
  {
    throw(cv_bridge::Exception("Error converting compressed image!"));
  }
  return img_ptr->image;
}

cv::Mat realm::to_realm::imageCompressed(const sensor_msgs::CompressedImage &msg)
{
  cv_bridge::CvImagePtr img_ptr;
  try
  {
    img_ptr = cv_bridge::toCvCopy(msg, "bgra8");
  }
  catch(...)
  {
    throw(cv_bridge::Exception("Error converting compressed image!"));
  }
  return img_ptr->image;
}

realm::CvGridMap::Ptr to_realm::cvGridMap(const realm_msgs::CvGridMap &msg)
{
  auto map = std::make_shared<realm::CvGridMap>();
  map->setGeometry(cv::Rect2d(msg.pos.x, msg.pos.y, msg.length_x, msg.length_y), msg.resolution);
  for (uint32_t i = 0; i < msg.layers.size(); ++i)
    map->add(msg.layers[i],  to_realm::image(msg.data[i]));
  return map;
}

} // namespace realm