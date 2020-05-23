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

#ifndef OPENREALM_GRABBER_ROS_NODE_H
#define OPENREALM_GRABBER_ROS_NODE_H

#include <iostream>
#include <mutex>

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <realm_ros/conversions.h>
#include <realm_io/realm_import.h>
#include <realm_io/exif_import.h>
#include <realm_io/utilities.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <realm_msgs/Frame.h>
#include <realm_msgs/Pinhole.h>

namespace realm
{

class RosGrabberNode
{
  using ApproxTimePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::NavSatFix>;

public:
  explicit RosGrabberNode();

  void spin();

  bool isOkay();

private:

  // name identifier for the node
  std::string _id_node;

  // chosen processing profile
  std::string _profile;

  // topics
  std::string _topic_image;
  std::string _topic_gnss;
  std::string _topic_heading;
  std::string _topic_relative_altitude;
  std::string _topic_out;

  // paths
  std::string _path_package;
  std::string _path_profile;

  // filepaths
  std::string _file_settings_camera;

  int _nrof_frames_received;

  double _fps;

  std::mutex _mutex_heading;
  double _heading;

  std::mutex _mutex_relative_altitude;
  double _relative_altitude;

  ros::NodeHandle _nh;

  message_filters::Subscriber<sensor_msgs::Image> _sub_input_image;
  message_filters::Subscriber<sensor_msgs::NavSatFix> _sub_input_gnss;
  message_filters::Synchronizer<ApproxTimePolicy> _sync_topics;

  ros::Publisher _pub_frame;

  camera::Pinhole::Ptr _cam;
  realm_msgs::Pinhole _cam_msg;

  void readParameters();
  void setPaths();

  void subHeading(const std_msgs::Float64 &msg);
  void subRelativeAltitude(const std_msgs::Float64 &msg);
  void subImageGnss(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::NavSatFixConstPtr &gnss);

  void publish(const Frame::Ptr &frame);
};

} // namespace realm

#endif //OPENREALM_GRABBER_ROS_NODE_H
