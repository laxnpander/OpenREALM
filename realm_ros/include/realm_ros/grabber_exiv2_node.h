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

#ifndef PROJECT_GRABBER_EXIV_NODE_H
#define PROJECT_GRABBER_EXIV_NODE_H

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>

#include <realm_ros/conversions.h>
#include <realm_io/realm_import.h>
#include <realm_io/exif_import.h>
#include <realm_io/utilities.h>

#include <sensor_msgs/Image.h>
#include <realm_msgs/Frame.h>
#include <realm_msgs/Pinhole.h>

namespace realm
{

/*!
 * @brief Simple "image from folder"-grabber for Exiv2 tagged images to convert into REALM input frames. Images can be
 * added dynamically, however the order accoding to the image names should be considered. Therefore DON'T add images in
 * wrong order. Additionally a pose file in TUM format can be provided to feed external pose data into the frame.
 * Exiv2 tags should at least contain the GNSS position. Tested name format is name_000000.suffix
 */

class Exiv2GrabberNode
{
  public:
    Exiv2GrabberNode();
    void spin();
    bool isOkay();
  private:

    bool _do_set_all_keyframes;

    // External pose can be provided in the format of a TUM trajectory file
    bool _use_apriori_pose;
    std::string _filepath_poses;
    std::unordered_map<uint64_t, cv::Mat> _poses;

    // External georeference can be provided in YAML format
    bool _use_apriori_georeference;
    std::string _filepath_georeference;
    cv::Mat _georeference;

    // External surface points can be provided in simple txt format with x, y, z
    bool _use_apriori_surface_pts;
    std::string _filepath_surface_pts;
    cv::Mat _surface_pts;

    double _fps;

    std::string _id_node;

    std::string _profile;
    std::string _topic_prefix;

    std::string _path_grab;
    std::string _path_package;
    std::string _path_profile;

    std::string _file_settings_camera;

    io::Exiv2FrameReader _exiv2_reader;

    ros::NodeHandle _nh;
    ros::Publisher _pub_frame;
    ros::Publisher _pub_image;

    camera::Pinhole::Ptr _cam;
    realm_msgs::Pinhole _cam_msg;

    size_t _id_curr_file;
    std::vector<std::string> _file_list;

    void readParams();
    void setPaths();
    void pubFrame(const Frame::Ptr &frame);
    std::vector<std::string> getFileList(const std::string& path);
};

} // namespace realm

#endif //PROJECT_GRABBER_EXIV_NODE
