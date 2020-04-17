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

#ifndef PROJECT_CONVERSION_H
#define PROJECT_CONVERSION_H

#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>
#include <pcl_ros/point_cloud.h>

#include <realm_core/enums.h>
#include <realm_core/frame.h>
#include <realm_core/utm32.h>
#include <realm_core/structs.h>
#include <realm_core/camera.h>
#include <realm_core/cv_grid_map.h>
#include <realm_core/analysis.h>

#include <std_msgs/Float32.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <realm_msgs/Frame.h>
#include <realm_msgs/Pinhole.h>
#include <realm_msgs/CvGridMap.h>
#include <realm_msgs/Georeference.h>
#include <realm_msgs/GroundImageCompressed.h>

namespace realm
{
namespace to_ros
{

/*!
 * @brief RVIZ has a bug, that is related to UTM coordinates being too big for Ogre float display. In result the mesh
 *        flickers when rotated etc. To fix it, we transform the mesh/triangle list to a nearby frame directly.
 * @param faces List of faces to be transformed
 * @param T Transformation mat
 * @return List of faces with fixed coordinates -> transformed to a nearby frame
 */
std::vector<Face> fixRvizMeshFlickerBug(const std::vector<Face> &faces, const tf::Transform &T);

/*!
 * @brief Converter from realm image cv::Mat to cv_bridge image type. Keeps the color encoding original
 * @param header Header of message with timestamp and frame_id
 * @param cv_img OpenCV type image cv::Mat
 * @return cv_bridge type image CvImage
 */
cv_bridge::CvImage image(const std_msgs::Header &header, const cv::Mat &cv_img);

/*!
 * @brief Converter from realm image cv::Mat to cv_bridge image type for display purposes. Changes the color encoding
 *        if necessary, because image_viewer does not accept all formats.
 * @param header Header of message with timestamp and frame_id
 * @param cv_img OpenCV type image cv::Mat
 * @return cv_bridge type image CvImage
 */
cv_bridge::CvImage imageDisplay(const std_msgs::Header &header, const cv::Mat &cv_img);

/*!
 * @brief Converter from realm utm to ros geodesy utm type.
 * @param r_utm Realm utm pose with easting, northing, altitude and heading
 * @return ROS / geodesy utm coordinate with easting, northing, altitude
 */
geodesy::UTMPoint utm(const realm::UTMPose &r_utm);

/*!
 * @brief Converter for realm utm coordinates to ros wgs84
 * @param Realm utm coordinate with easting, northing, altitude and heading
 * @return ROS / geodesy wgs84 point with latitude, longitude, altitude
 */
geographic_msgs::GeoPoint wgs84(const realm::UTMPose &r_utm);

/*!
 * @brief Converter for ROS pose to ROS transform. Mainly used for visualization
 * @param msg Geometry msg in ROS type pose msg
 * @return Transform
 */
tf::Transform tf(const geometry_msgs::Pose &msg);

/*!
 * @brief Converter for OpenCV/REALM pose to ROS transform.
 * @param cv_pose Currently only (3x4) matrix (R|t) containing pose information supported.
 * @return ROS transform
 */
tf::Transform tf(const cv::Mat &cv_pose);

/*!
 * @brief Converter for OpenCV/REALM matrix to 4x4 matrix. Is used for homogenous transformation, because ROS tf can
 *        not handle scale in the Rotation
 * @param mat 4x4 matrix
 * @return Array arranged as linear vector
 */
realm_msgs::Georeference georeference(const cv::Mat &mat);

/*!
 * @brief Converter for OpenCV/REALM transformation to ROS geometry message
 * @param T Transformation as cv::Mat
 * @return ROS transform message
 */
geometry_msgs::Transform tfMsg(const cv::Mat &T);

/*!
 * @brief Converter for ROS pose to ROS transform with timestamp. Mainly used for visualization
 * @param msg Geometry msg in ROS type pose msg
 * @return Transform with current timestamp
 */
tf::StampedTransform tfStamped(const geometry_msgs::PoseStamped &msg);

/*!
 * @brief Converter for realm point cloud as cv::Mat with row(i) = (x,y,z,r,g,b,nx,ny,nz) where at least nx,ny,nz are
 *        optional.
 * @param header Header for ROS message
 * @param points Point cloud as cv::Mat
 * @return ROS message point cloud
 */
sensor_msgs::PointCloud2 pointCloud(const std_msgs::Header &header, const cv::Mat &points);

/*!
 * @brief Converter for realm pinhole camera to ROS message
 * @param cam Pinhole camera model in realm type
 * @return ROS message pinhole camera
 */
realm_msgs::Pinhole pinhole(const realm::camera::Pinhole::Ptr &cam);

/*!
 * @brief Converter for a georeferenced image to a ROS type ground image. The latter is used in RVIZ by ground image
 *        plugin to display a 2D image at ENU coordinate frame
 * @param header Header for ROS message
 * @param img OpenCV image type cv::mat
 * @param ulc Upper left corner of the image to be displayed in global ENU coordinate frame (UTM)
 * @param GSD Resolution / ground sampling distance of the image
 * @param mask Mask might be set to only extract a certain valid region of img
 * @return ROS message ground image
 */
realm_msgs::GroundImageCompressed groundImage(const std_msgs::Header &header,
                                              const cv::Mat &img,
                                              const realm::UTMPose &ulc,
                                              double GSD,
                                              const cv::Mat &mask = cv::Mat());

/*!
 * @brief Converter for realm/OpenCV pose to ROS geometry message. Pose M is defined as 3x4 matrix with
 *         M = (R t)
 *         and 3x3 R rotation mat, t 3x1 translation vector (x = Easting, y = Northing, z = Altitude)
 * @param cv_pose (3x4) opencv mat type
 * @return ROS message pose with translation (x = Easting, y = Northing, z = Altitude)
 */
geometry_msgs::Pose pose(const cv::Mat &cv_pose);

/*!
 * @brief Converter for TF transform to ROS geometry message. Pose M is defined as 3x4 matrix with
 *         M = (R t)
 *         and 3x3 R rotation mat, t 3x1 translation vector (x = Easting, y = Northing, z = Altitude)
 * @param transform (3x4) tf::transform with quaternion and translation
 * @return ROS message pose with translation (x = Easting, y = Northing, z = Altitude)
 */
geometry_msgs::Pose pose(const tf::Transform &transform);

/*!
 * @brief Converter for realm/OpenCV pose to ROS geometry message. Pose M is defined as 3x4 matrix with
 *         M = (R t)
 *         and 3x3 R rotation mat, t 3x1 translation vector (x = Easting, y = Northing, z = Altitude)
 * @param cv_pose (3x4) opencv mat type
 * @param zone Zone of the utm translation vector
 * @param band Band of the utm translation vector
 * @return ROS message pose with translation (x = Easting, y = Northing, z = Altitude)
 */
geometry_msgs::Pose poseWgs84(const cv::Mat &cv_pose, uint8_t zone, char band);

/*!
 * @brief Converter for realm CvGridMap to ROS message
 * @param header Header for ROS message
 * @param map CvGridMap to be converted to a ROS message
 * @return ROS message of CvGridMap
 */
realm_msgs::CvGridMap cvGridMap(const std_msgs::Header &header, const realm::CvGridMap::Ptr &map);

/*!
 * @brief Converter for realm frame to ROS message. "Frame" is the basic type exchanged between different stages and
 *        contains als measured and generated data.
 * @param header Desired header of the ROS message
 * @param frame Pointer to frame
 * @return ROS message of Frame
 */
realm_msgs::Frame frame(const std_msgs::Header &header, const realm::Frame::Ptr &frame);

/*!
 * @brief Converter for realm mesh (vector of faces) to ROS visualization message.
 * @param header Desired header of the ROS message
 * @param faces Vector of triangles (faces)
 * @param ns Namespace of the mesh to be published
 * @param id Id of the mesh to be published
 * @param type Type of mesh, currently designed for triangle marker only
 * @param action Action to take for the mesh (update, add, ...)
 * @return ROS message of mesh
 */
visualization_msgs::Marker meshMarker(const std_msgs::Header &header,
                                      const std::vector<Face> &faces,
                                      const std::string &ns,
                                      int32_t id,
                                      int32_t type,
                                      int32_t action,
                                      const tf::Transform &T);

} // namespace to_ros

namespace to_realm
{

/*!
 * @brief Converter for ROS wgs84 coordinate with heading to realm UTMPose
 * @param gnss Global position with Lat, lon, alt
 * @param heading Heading info of the camera
 * @return REALM utm type
 */
UTMPose utm(const sensor_msgs::NavSatFix &gnss, const std_msgs::Float32 &heading = std_msgs::Float32());

/*!
 * @brief Converter for ROS frame message to realm type. "Frame" is the basic type exchanged between different stages and
 *        contains als measured and generated data.
 * @param msg ROS Frame message
 * @return REALM frame type
 */
Frame::Ptr frame(const realm_msgs::Frame &msg);

/*!
 * @brief Converter for ROS pinhole message to realm type. Contains the basic camera model
 * @param msg ROS pinhole message
 * @return REALM camera pinhole type
 */
camera::Pinhole::Ptr pinhole(const realm_msgs::Pinhole &msg);

/*!
 * @brief Converter for ROS pose message to opencv/REALM pose type.
 * @param ros_pose ROS pose with R and t
 * @return (3x4) mat with M = (R|t)
 */
cv::Mat pose(const geometry_msgs::Pose &ros_pose);

/*!
 * @brief Converter for tf::transform to opencv/REALM pose type.
 * @param ros_pose tf::transform with R and t
 * @return (3x4) mat with M = (R|t)
 */
cv::Mat pose(const tf::Transform &transform);

/*!
 * @brief Converter for geometry_msgs::Transform to opencv/REALM 4x4 pose mat
 * @param msg geometry_msgs::Transform with R and t
 * @return (4x4) mat with
 *          (R | t)
 *          (0 | 1)
 */
cv::Mat tf(const geometry_msgs::Transform &msg);

/*!
 * @brief Converter for std_msgs::Float64MultiArray to opencv/REALM 4x4 matrix
 * @param msg std_msgs::Float64MultiArray containing 16 elements
 * @return (4x4) mat
 */
cv::Mat georeference(const realm_msgs::Georeference &msg);

/*!
 * @brief Converter for ROS point cloud as cv_bridge msg to opencv/REALM mat with row(i) = (x,y,z,r,g,b,nx,ny,nz),
 *        where at least nx,ny,nz are optional.
 * @param msg ROS message for cv_bridge mat type
 * @return REALM point cloud with dim (nx3) or (nx9)
 */
cv::Mat pointCloud(const sensor_msgs::Image &msg);

/*!
 * @brief Converter for ROS image message to opencv/REALM mat type
 * @param msg ROS message of image
 * @return REALM image
 */
cv::Mat image(const sensor_msgs::Image &msg);

/*!
 * @brief Converter for ROS image compressed message to opencv/REALM mat type
 * @param msg ROS message of compressed image
 * @return REALM image
 */
cv::Mat imageCompressed(const sensor_msgs::CompressedImage &msg);

/*!
 * @brief Converter for ROS CvGridMap message to REALM CvGridMap type
 * @param msg ROS message of CvGridMap
 * @return REALM CvGridMap
 */
realm::CvGridMap::Ptr cvGridMap(const realm_msgs::CvGridMap &msg);

} // namespace to_realm
} // namespace realm

#endif