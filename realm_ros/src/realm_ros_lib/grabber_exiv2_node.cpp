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

#include <realm_ros/grabber_exiv2_node.h>

using namespace realm;

Exiv2GrabberNode::Exiv2GrabberNode()
: _do_set_all_keyframes(false),
  _use_apriori_pose(false),
  _use_apriori_georeference(false),
  _use_apriori_surface_pts(false),
  _fps(0.0),
  _id_curr_file(0),
  _cam(nullptr)
{
  readParams();
  setPaths();

  _exiv2_reader = io::Exiv2FrameReader(io::Exiv2FrameReader::FrameTags::loadFromFile(_path_profile + "/config/exif.yaml"));

  // Loading camera from provided file, check first if absolute path was provided
  if (io::fileExists(_file_settings_camera))
    _cam = std::make_shared<camera::Pinhole>(io::loadCameraFromYaml(_file_settings_camera));
  else
    _cam = std::make_shared<camera::Pinhole>(io::loadCameraFromYaml(_path_profile + "/camera/", _file_settings_camera));
  _cam_msg = to_ros::pinhole(_cam);

  // Loading poses from file if provided
  if (_use_apriori_pose)
  {
    _poses = io::loadTrajectoryFromTxtTUM(_filepath_poses);
    ROS_INFO_STREAM("Succesfully loaded " << _poses.size() << " external poses.");
  }

  // Loading georeference from file if provided
  if (_use_apriori_georeference)
  {
    _georeference = io::loadGeoreferenceFromYaml(_filepath_georeference);
    ROS_INFO_STREAM("Succesfully loaded external georeference:\n" << _georeference);
  }

  // Loading surface points from file if provided
  if (_use_apriori_surface_pts)
  {
    _surface_pts = io::loadSurfacePointsFromTxt(_filepath_surface_pts);
    ROS_INFO_STREAM("Succesfully loaded " << _surface_pts.rows << " external surface points.");
  }

  ROS_INFO_STREAM(
      "Exiv2 Grabber Node [Ankommen]: Successfully loaded camera: "
          << "\n\tcx = " << _cam->cx()
          << "\n\tcy = " << _cam->cy()
          << "\n\tfx = " << _cam->fx()
          << "\n\tfy = " << _cam->fy()
          << "\n\tk1 = " << _cam->k1()
          << "\n\tk2 = " << _cam->k2()
          << "\n\tp1 = " << _cam->p1()
          << "\n\tp2 = " << _cam->p2());

  // ROS related inits
  _topic_prefix = "/realm/" + _id_node;
  _pub_frame = _nh.advertise<realm_msgs::Frame>(_topic_prefix + "/input", 5);
  _pub_image = _nh.advertise<sensor_msgs::Image>(_topic_prefix + "/img", 5);

  // Start grabbing images
  _file_list = getFileList(_path_grab);
}

void Exiv2GrabberNode::readParams()
{
  ros::NodeHandle param_nh("~");

  param_nh.param("config/id", _id_node, std::string("uninitialised"));
  param_nh.param("config/input", _path_grab, std::string("uninitialised"));
  param_nh.param("config/rate", _fps, 0.0);
  param_nh.param("config/profile", _profile, std::string("uninitialised"));
  param_nh.param("config/opt/poses", _filepath_poses, std::string("uninitialised"));
  param_nh.param("config/opt/georeference", _filepath_georeference, std::string("uninitialised"));
  param_nh.param("config/opt/surface_pts", _filepath_surface_pts, std::string("uninitialised"));
  param_nh.param("config/opt/set_all_keyframes", _do_set_all_keyframes, false);

  if (_fps < 0.01)
    throw(std::invalid_argument("Error reading exiv2 grabber parameters: Frame rate is too low!"));
  if (_filepath_poses != "uninitialised")
    _use_apriori_pose = true;
  if (_filepath_georeference != "uninitialised")
    _use_apriori_georeference = true;
  if (_filepath_surface_pts != "uninitialised")
    _use_apriori_surface_pts = true;
}

void Exiv2GrabberNode::setPaths()
{
  _path_package = ros::package::getPath("realm_ros");
  _path_profile = _path_package + "/profiles/" + _profile;

  _file_settings_camera = _path_profile + "/camera/calib.yaml";

  if (!io::dirExists(_path_profile))
    throw(std::invalid_argument("Error: Config folder path '" + _path_profile + "' does not exist!"));
  if (!io::dirExists(_path_grab))
    throw(std::invalid_argument("Error grabbing Exiv2 images: Folder '" + _path_grab + "' does not exist!"));
}

void Exiv2GrabberNode::spin()
{
  ros::Rate rate(_fps);
  if (_id_curr_file < _file_list.size())
  {
    ROS_INFO_STREAM("Image #" << _id_curr_file << ", image Path: " << _file_list[_id_curr_file]);
    Frame::Ptr frame = _exiv2_reader.loadFrameFromExiv2(_id_node, _cam, _file_list[_id_curr_file]);

    // External pose can be provided
    if (_use_apriori_pose)
    {
      cv::Mat pose = _poses[frame->getTimestamp()];
      if (pose.empty())
        throw(std::runtime_error("Error adding external pose informations: No pose was found. Maybe images or provided pose file do not match?"));
      frame->setVisualPose(pose);
      if (_do_set_all_keyframes)
        frame->setKeyframe(true);
    }

    if (_use_apriori_georeference)
    {
      frame->updateGeoreference(_georeference);
    }

    // External surface points can be provided
    if (_use_apriori_surface_pts)
    {
      if (_surface_pts.empty())
        throw(std::runtime_error("Error adding external surface points: No points were found!"));
      frame->setSurfacePoints(_surface_pts);
    }

    pubFrame(frame);
    _id_curr_file++;
  }

  std::vector<std::string> new_file_list = getFileList(_path_grab);
  if (new_file_list.size() != _file_list.size())
  {
    ROS_INFO_STREAM("Processed images in folder: " << _file_list.size() << " / " << new_file_list.size());
    std::set_difference(new_file_list.begin(), new_file_list.end(), _file_list.begin(), _file_list.end(), std::back_inserter(_file_list));
    std::sort(_file_list.begin(), _file_list.end());
  }

  rate.sleep();
}

void Exiv2GrabberNode::pubFrame(const Frame::Ptr &frame)
{
  // Create message header
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "utm";

  // Create message informations
  realm_msgs::Frame msg_frame = to_ros::frame(header, frame);
  sensor_msgs::Image msg_img = *to_ros::imageDisplay(header, frame->getImageRaw()).toImageMsg();

  _pub_frame.publish(msg_frame);
  _pub_image.publish(msg_img);
}

bool Exiv2GrabberNode::isOkay()
{
  return _nh.ok();
}

std::vector<std::string> Exiv2GrabberNode::getFileList(const std::string& path)
{
  std::vector<std::string> file_names;
  if (!path.empty())
  {
    boost::filesystem::path apk_path(path);
    boost::filesystem::recursive_directory_iterator end;

    for (boost::filesystem::recursive_directory_iterator it(apk_path); it != end; ++it)
    {
      const boost::filesystem::path cp = (*it);
      file_names.push_back(cp.string());
    }
  }
  std::sort(file_names.begin(), file_names.end());
  return file_names;
}