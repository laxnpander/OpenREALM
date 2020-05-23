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

#include <realm_ros/grabber_ros_node.h>

using namespace realm;

RosGrabberNode::RosGrabberNode()
  : _nrof_frames_received(0),
    _fps(0.0),
    _cam(nullptr),
    _sync_topics(ApproxTimePolicy(1000), _sub_input_image, _sub_input_gnss)
{
  readParameters();
  setPaths();

  // Loading camera from provided file, check first if absolute path was provided
  if (io::fileExists(_file_settings_camera))
    _cam = std::make_shared<camera::Pinhole>(io::loadCameraFromYaml(_file_settings_camera, &_fps));
  else
    _cam = std::make_shared<camera::Pinhole>(io::loadCameraFromYaml(_path_profile + "/camera/", _file_settings_camera));
  _cam_msg = to_ros::pinhole(_cam);

  ROS_INFO_STREAM(
          "ROS Grabber Node successfully loaded camera: "
                  << "\n\tfps = " << _fps
                  << "\n\tcx = " << _cam->cx()
                  << "\n\tcy = " << _cam->cy()
                  << "\n\tfx = " << _cam->fx()
                  << "\n\tfy = " << _cam->fy()
                  << "\n\tk1 = " << _cam->k1()
                  << "\n\tk2 = " << _cam->k2()
                  << "\n\tp1 = " << _cam->p1()
                  << "\n\tp2 = " << _cam->p2()
                  << "\n\tk3 = " << _cam->k3());

  _nh.subscribe(_topic_heading, 10, &RosGrabberNode::subHeading, this, ros::TransportHints());
  _nh.subscribe(_topic_relative_altitude, 10, &RosGrabberNode::subRelativeAltitude, this, ros::TransportHints());
  _sub_input_image.subscribe(_nh, _topic_image, 10);
  _sub_input_gnss.subscribe(_nh, _topic_gnss, 10);
  _sync_topics.registerCallback(boost::bind(&RosGrabberNode::subImageGnss, this, _1, _2));

  _pub_frame = _nh.advertise<realm_msgs::Frame>(_topic_out, 100);

  ROS_INFO_STREAM("ROS Grabber Node subscribed to topics:\n"
                  "- Image topic:\t\t"  << _topic_image   << "\n"
                  "- GNSS topic:\t\t"   << _topic_gnss    << "\n"
                  "- Heading topic:\t"  << _topic_heading << "\n"
                  "- Rel. Altitude topic:  "  << _topic_relative_altitude << "\n"
                  "- Output topic:\t\t" << _topic_out     << "\n");
}

void RosGrabberNode::readParameters()
{
  ros::NodeHandle param_nh("~");

  param_nh.param("config/id",               _id_node,                 std::string("uninitialised"));
  param_nh.param("config/profile",          _profile,                 std::string("uninitialised"));
  param_nh.param("topic/image",             _topic_image,             std::string("uninitialised"));
  param_nh.param("topic/gnss",              _topic_gnss,              std::string("uninitialised"));
  param_nh.param("topic/heading",           _topic_heading,           std::string("uninitialised"));
  param_nh.param("topic/relative_altitude", _topic_relative_altitude, std::string("uninitialised"));
  param_nh.param("topic/out",               _topic_out,               std::string("uninitialised"));
}

void RosGrabberNode::setPaths()
{
  _path_package = ros::package::getPath("realm_ros");
  _path_profile = _path_package + "/profiles/" + _profile;

  _file_settings_camera = _path_profile + "/camera/calib.yaml";

  if (!io::dirExists(_path_profile))
    throw(std::invalid_argument("Error: Config folder path '" + _path_profile + "' does not exist!"));
}

void RosGrabberNode::spin()
{
  ros::Rate rate(1.0);
  rate.sleep();
}

bool RosGrabberNode::isOkay()
{
  return _nh.ok();
}

void RosGrabberNode::subHeading(const std_msgs::Float64 &msg)
{
  _mutex_heading.lock();
  _heading = msg.data;
  _mutex_heading.unlock();
}

void RosGrabberNode::subRelativeAltitude(const std_msgs::Float64 &msg)
{
  _mutex_relative_altitude.lock();
  _relative_altitude = msg.data;
  _mutex_relative_altitude.unlock();
}

void RosGrabberNode::subImageGnss(const sensor_msgs::ImageConstPtr &msg_img, const sensor_msgs::NavSatFixConstPtr &msg_gnss)
{
  cv::Mat img = to_realm::image(*msg_img);

  _mutex_relative_altitude.lock();
  _mutex_heading.lock();

  WGSPose wgs;
  wgs.latitude = msg_gnss->latitude;
  wgs.longitude = msg_gnss->longitude;
  wgs.altitude = msg_gnss->altitude;
  wgs.heading = _heading;

  UTMPose utm = gis::convertToUTM(wgs);

  _mutex_heading.unlock();
  _mutex_relative_altitude.unlock();

  auto frame = std::make_shared<Frame>(_id_node, _nrof_frames_received, msg_img->header.stamp.sec, img, utm, _cam);

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "/realm";
  _pub_frame.publish(to_ros::frame(header, frame));

  _nrof_frames_received++;
}