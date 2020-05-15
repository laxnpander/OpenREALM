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

#include <realm_ros/stage_node.h>

using namespace realm;

StageNode::StageNode(int argc, char **argv)
: _nrof_msgs_rcvd(0),
  _is_master_stage(false),
  _do_shutdown(false),
  _is_tf_base_initialized(false),
  _is_tf_stage_initialized(false)
{
  // Read basic launch file inputs
  readParams();

  // Specify stage
  setPaths();
  readStageSettings();

  // Set naming conventions
  _topic_prefix = "/realm/" + _id_camera + "/" + _type_stage + "/";
  _tf_base_frame_name = "realm_base";
  _tf_stage_frame_name = "realm_" + _id_camera + "_" + _type_stage;

  // Set ros subscriber according to launch input
  _sub_input_frame = _nh.subscribe(_topic_frame_in, 5, &StageNode::subFrame, this, ros::TransportHints());
  if (_is_master_stage)
  {
    _publisher.insert({"general/output_dir", _nh.advertise<std_msgs::String>("/realm/" + _id_camera + "/general/output_dir", 5)});
    _publisher.insert({"general/gnss_base", _nh.advertise<sensor_msgs::NavSatFix>("/realm/" + _id_camera + "/general/gnss_base", 5)});
  }
  else
    _sub_output_dir = _nh.subscribe("/realm/"+ _id_camera +"/general/output_dir", 5, &StageNode::subOutputPath, this, ros::TransportHints());

  // Set ros services for stage handling
  _srv_req_finish = _nh.advertiseService(_topic_prefix + "request_finish", &StageNode::srvFinish, this);
  _srv_req_stop = _nh.advertiseService(_topic_prefix + "request_stop", &StageNode::srvStop, this);
  _srv_req_resume = _nh.advertiseService(_topic_prefix + "request_resume", &StageNode::srvResume, this);
  _srv_req_reset = _nh.advertiseService(_topic_prefix + "request_reset", &StageNode::srvReset, this);
  _srv_change_param = _nh.advertiseService(_topic_prefix + "change_param", &StageNode::srvChangeParam, this);

  // Provide camera information a priori to all stages
  ROS_INFO("STAGE_NODE [%s]: : Loading camera from path:\n\t%s", _type_stage.c_str(),_file_settings_camera.c_str());
  _settings_camera = CameraSettingsFactory::load(_file_settings_camera);
  ROS_INFO("STAGE_NODE [%s]: : Detected camera model: '%s'", _type_stage.c_str(), (*_settings_camera)["type"].toString().c_str());

  // Create stages
  if (_type_stage == "pose_estimation")
    createStagePoseEstimation();
  if (_type_stage == "densification")
    createStageDensification();
  if (_type_stage == "surface_generation")
    createStageSurfaceGeneration();
  if (_type_stage == "ortho_rectification")
    createStageOrthoRectification();
  if (_type_stage == "mosaicing")
    createStageMosaicing();

  // set stage path if master stage
  if (_is_master_stage)
    _stage->initStagePath(_path_output + "/" + _dir_output);

  // Start the thread for processing
  _stage->start();
  ROS_INFO("STAGE_NODE [%s]: Started stage node successfully!", _type_stage.c_str());
}

StageNode::~StageNode()
{
  // In case of an unproper shutdown, at least call the finish procedure
  if (!_do_shutdown)
  {
    _stage->requestFinish();
    _stage->join();
  }
}

void StageNode::spin()
{
  if (_is_master_stage)
  {
    // Share output folder with slaves
    std_msgs::String msg;
    msg.data = _dir_output;
    _publisher["general/output_dir"].publish(msg);
  }

  static tf::TransformBroadcaster br;
  if (_is_tf_base_initialized && _is_master_stage)
  {
    // Master stage sends first tf as mission reference
    br.sendTransform(tf::StampedTransform(_tf_base, ros::Time::now(), "utm", _tf_base_frame_name));
    _publisher["general/gnss_base"].publish(_gnss_base);
  }

  if (_is_tf_stage_initialized)
  {
    // Publish of current mission reference
    br.sendTransform(tf::StampedTransform(_tf_stage, ros::Time::now(), _tf_base_frame_name, _tf_stage_frame_name));
  }
}

bool StageNode::isOkay()
{
  std::unique_lock<std::mutex> lock(_mutex_do_shutdown);
  return (!_do_shutdown && _nh.ok());
}

void StageNode::createStagePoseEstimation()
{
  // Pose estimation uses external frameworks, therefore load settings for that
  ROS_INFO("STAGE_NODE [%s]: : Loading vslam settings from path:\n\t%s", _type_stage.c_str(), _file_settings_method.c_str());
  VisualSlamSettings::Ptr settings_vslam = VisualSlamSettingsFactory::load(_file_settings_method, _path_profile + "/" + _type_stage + "/method");
  ROS_INFO("STAGE_NODE [%s]: : Detected vslam type: '%s'", _type_stage.c_str(), (*settings_vslam)["type"].toString().c_str());

  // Topic and stage creation
  _stage = std::make_shared<stages::PoseEstimation>(_settings_stage, settings_vslam, _settings_camera, (*_settings_camera)["fps"].toDouble());
  _publisher.insert({"output/frame", _nh.advertise<realm_msgs::Frame>(_topic_frame_out, 5)});
  _publisher.insert({"output/pose/visual/utm", _nh.advertise<geometry_msgs::PoseStamped>(_topic_prefix + "pose/visual/utm", 5)});
  _publisher.insert({"output/pose/visual/wgs", _nh.advertise<geometry_msgs::PoseStamped>(_topic_prefix + "pose/visual/wgs", 5)});
  _publisher.insert({"output/pose/visual/traj", _nh.advertise<nav_msgs::Path>(_topic_prefix + "pose/visual/traj", 5)});
  _publisher.insert({"output/pose/gnss/utm", _nh.advertise<geometry_msgs::PoseStamped>(_topic_prefix + "pose/gnss/utm", 5)});
  _publisher.insert({"output/pose/gnss/wgs", _nh.advertise<geometry_msgs::PoseStamped>(_topic_prefix + "pose/gnss/wgs", 5)});
  _publisher.insert({"output/pose/gnss/traj", _nh.advertise<nav_msgs::Path>(_topic_prefix + "pose/gnss/traj", 5)});
  _publisher.insert({"output/pointcloud", _nh.advertise<sensor_msgs::PointCloud2>(_topic_prefix + "pointcloud", 5)});
  _publisher.insert({"debug/tracked", _nh.advertise<sensor_msgs::Image>(_topic_prefix + "tracked", 5)});
  linkStageTransport();
}

void StageNode::createStageDensification()
{
  // Densification uses external frameworks, therefore load settings for that
  ROS_INFO("STAGE_NODE [%s]: : Loading densifier settings from path:\n\t%s", _type_stage.c_str(), _file_settings_method.c_str());
  DensifierSettings::Ptr settings_densifier = DensifierSettingsFactory::load(_file_settings_method, _path_profile + "/" + _type_stage + "/method");
  ROS_INFO("STAGE_NODE [%s]: : Detected densifier type: '%s'", _type_stage.c_str(), (*settings_densifier)["type"].toString().c_str());

  // Topic and stage creation
  _stage = std::make_shared<stages::Densification>(_settings_stage, settings_densifier, (*_settings_camera)["fps"].toDouble());
  _publisher.insert({"output/frame", _nh.advertise<realm_msgs::Frame>(_topic_frame_out, 5)});
  _publisher.insert({"output/pose/utm", _nh.advertise<geometry_msgs::PoseStamped>(_topic_prefix + "pose/utm", 5)});
  _publisher.insert({"output/pose/wgs", _nh.advertise<geometry_msgs::PoseStamped>(_topic_prefix + "pose/wgs", 5)});
  _publisher.insert({"output/pointcloud", _nh.advertise<sensor_msgs::PointCloud2>(_topic_prefix + "pointcloud", 5)});
  _publisher.insert({"output/img_rectified", _nh.advertise<sensor_msgs::Image>(_topic_prefix + "img", 5)});
  _publisher.insert({"output/depth", _nh.advertise<sensor_msgs::Image>(_topic_prefix + "depth", 5)});
  _publisher.insert({"output/depth_display", _nh.advertise<sensor_msgs::Image>(_topic_prefix + "depth_display", 5)});
  linkStageTransport();
}

void StageNode::createStageSurfaceGeneration()
{
  _stage = std::make_shared<stages::SurfaceGeneration>(_settings_stage, (*_settings_camera)["fps"].toDouble());
  _publisher.insert({"output/frame", _nh.advertise<realm_msgs::Frame>(_topic_frame_out, 5)});
  _publisher.insert({"output/elevation_map", _nh.advertise<sensor_msgs::Image>(_topic_prefix + "elevation_map", 5)});
  linkStageTransport();
}

void StageNode::createStageOrthoRectification()
{
  _stage = std::make_shared<stages::OrthoRectification>(_settings_stage, (*_settings_camera)["fps"].toDouble());
  _publisher.insert({"output/frame", _nh.advertise<realm_msgs::Frame>(_topic_frame_out, 5)});
  _publisher.insert({"output/rectified", _nh.advertise<sensor_msgs::Image>(_topic_prefix + "rectified", 5)});
  _publisher.insert({"output/pointcloud", _nh.advertise<sensor_msgs::PointCloud2>(_topic_prefix + "pointcloud", 5)});
  linkStageTransport();
}

void StageNode::createStageMosaicing()
{
  _stage = std::make_shared<stages::Mosaicing>(_settings_stage, (*_settings_camera)["fps"].toDouble());
  _publisher.insert({"output/rgb", _nh.advertise<sensor_msgs::Image>(_topic_prefix + "rgb", 5)});
  _publisher.insert({"output/elevation", _nh.advertise<sensor_msgs::Image>(_topic_prefix + "elevation", 5)});
  _publisher.insert({"output/pointcloud", _nh.advertise<sensor_msgs::PointCloud2>(_topic_prefix + "pointcloud", 5)});
  _publisher.insert({"output/mesh", _nh.advertise<visualization_msgs::Marker>(_topic_prefix + "mesh", 5)});
  _publisher.insert({"output/update/ortho", _nh.advertise<realm_msgs::GroundImageCompressed>(_topic_prefix + "update/ortho", 5)});
  //_publisher.insert({"output/update/elevation", _nh.advertise<realm_msgs::GroundImageCompressed>(_topic_prefix + "update/elevation", 5)});
  linkStageTransport();
}

void StageNode::linkStageTransport()
{
  namespace ph = std::placeholders;
  auto transport_frame = std::bind(&StageNode::pubFrame, this, ph::_1, ph::_2);
  auto transport_pose = std::bind(&StageNode::pubPose, this, ph::_1, ph::_2, ph::_3, ph::_4);
  auto transport_pointcloud = std::bind(&StageNode::pubPointCloud, this, ph::_1, ph::_2);
  auto transport_img = std::bind(&StageNode::pubImage, this, ph::_1, ph::_2);
  auto transport_depth = std::bind(&StageNode::pubDepthMap, this, ph::_1, ph::_2);
  auto transport_mesh = std::bind(&StageNode::pubMesh, this, ph::_1, ph::_2);
  auto transport_cvgridmap = std::bind(&StageNode::pubCvGridMap, this, ph::_1, ph::_2, ph::_3, ph::_4);
  _stage->registerFrameTransport(transport_frame);
  _stage->registerPoseTransport(transport_pose);
  _stage->registerPointCloudTransport(transport_pointcloud);
  _stage->registerImageTransport(transport_img);
  _stage->registerDepthMapTransport(transport_img);
  _stage->registerMeshTransport(transport_mesh);
  _stage->registerCvGridMapTransport(transport_cvgridmap);
}

void StageNode::reset()
{
  // TODO: Currently reset of stage via service seems to not suite the node reset
  _nrof_msgs_rcvd = 0;
  _is_tf_base_initialized = false;
  _is_tf_stage_initialized = false;
}

void StageNode::subFrame(const realm_msgs::Frame &msg)
{
  ROS_INFO("STAGE_NODE [%s]: Received frame.", _type_stage.c_str());
  if (msg.do_reset.data)
  {
    _stage->requestReset();
    _publisher["output/geoimg"].publish(msg);
    ROS_WARN("STAGE_NODE [%s]: Mission has triggered reset. Stage resetting...", _type_stage.c_str());
    return;
  }

  Frame::Ptr frame = to_realm::frame(msg);
  if (_is_master_stage)
  {
    if (!_is_tf_base_initialized)
      setTfBaseFrame(frame->getGnssUtm());
  }

  _stage->addFrame(std::move(frame));
  _nrof_msgs_rcvd++;
}

void StageNode::subOutputPath(const std_msgs::String &msg)
{
  // check if output directory has changed
  if (_dir_output != msg.data)
  {
    // Note: master privilege is not to create folder, but to set the name of the folder
    _dir_output = msg.data;
    if (!io::dirExists(_dir_output))
      io::createDir(_dir_output);
    _stage->initStagePath(_path_output + "/" + _dir_output);

    // Debug info
    ROS_INFO("STAGE_NODE [%s]: Received output directory, set to:\n\t%s",
             _type_stage.c_str(),
             (_path_output + "/" + _dir_output).c_str());
  }
}

void StageNode::pubFrame(const Frame::Ptr &frame, const std::string &topic)
{
  ros::Publisher publisher = _publisher[topic];
  if (publisher.getNumSubscribers() == 0)
    return;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "utm";

  realm_msgs::Frame msg = to_ros::frame(header, frame);
  publisher.publish(msg);
  ROS_INFO("STAGE_NODE [%s]: Published frame.", _type_stage.c_str());
}

void StageNode::pubPose(const cv::Mat &pose, uint8_t zone, char band, const std::string &topic)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();

  // utm
  geometry_msgs::PoseStamped utm_msg;
  utm_msg.header = header;
  utm_msg.header.frame_id = "utm";
  utm_msg.pose = to_ros::pose(pose);

  // wgs
  geometry_msgs::PoseStamped wgs_msg;
  wgs_msg.header = header;
  wgs_msg.header.frame_id = "wgs";
  wgs_msg.pose = to_ros::poseWgs84(pose, zone, band);
  _publisher[topic + "/utm"].publish(utm_msg);
  _publisher[topic + "/wgs"].publish(wgs_msg);

  // trajectory
  std::vector<geometry_msgs::PoseStamped>* trajectory = &_trajectories[topic];
  trajectory->push_back(utm_msg);
  pubTrajectory(*trajectory, topic + "/traj");

  // transform
  _tf_stage = to_ros::tf(pose);
  if (!_is_tf_stage_initialized)
    _is_tf_stage_initialized = true;
}

void StageNode::pubPointCloud(const cv::Mat &pts, const std::string &topic)
{
  ros::Publisher publisher = _publisher[topic];
  if (publisher.getNumSubscribers() == 0)
    return;

  std_msgs::Header header;
  header.frame_id = "utm";
  header.stamp = ros::Time::now();
  sensor_msgs::PointCloud2 msg = to_ros::pointCloud(header, pts);
  publisher.publish(msg);
}

void StageNode::pubDepthMap(const cv::Mat &img, const std::string &topic)
{
  ros::Publisher publisher = _publisher[topic];
  if (publisher.getNumSubscribers() == 0)
    return;

  std_msgs::Header header;
  header.frame_id = "utm";
  header.stamp = ros::Time::now();

  sensor_msgs::Image msg;
  msg = *to_ros::image(header, img).toImageMsg();
  publisher.publish(msg);
}

void StageNode::pubImage(const cv::Mat &img, const std::string &topic)
{
  ros::Publisher publisher = _publisher[topic];
  if (publisher.getNumSubscribers() == 0)
    return;

  std_msgs::Header header;
  header.frame_id = "utm";
  header.stamp = ros::Time::now();

  sensor_msgs::Image msg;
  msg = *to_ros::imageDisplay(header, img).toImageMsg();
  publisher.publish(msg);
}

void StageNode::pubMesh(const std::vector<Face> &faces, const std::string &topic)
{
  std::cout << "blub1" << std::endl;
  std::unique_lock<std::mutex> lock(_mutex_do_shutdown);
  ros::Publisher publisher = _publisher[topic];
  if (publisher.getNumSubscribers() == 0)
    return;

  std_msgs::Header header;
  header.frame_id = _tf_base_frame_name;
  header.stamp = ros::Time::now();

  visualization_msgs::Marker msg = to_ros::meshMarker(header, faces, "Global Map", 0,
                                                      visualization_msgs::Marker::TRIANGLE_LIST,
                                                      visualization_msgs::Marker::ADD, _tf_base.inverse());
  publisher.publish(msg);
}

void StageNode::pubCvGridMap(const CvGridMap &map, uint8_t zone, char band, const std::string &topic)
{
  ros::Publisher publisher = _publisher[topic];
  if (publisher.getNumSubscribers() == 0)
    return;

  std_msgs::Header header;
  header.frame_id = "utm";
  header.stamp = ros::Time::now();

  cv::Rect2d roi = map.roi();
  realm::UTMPose utm;
  utm.easting = roi.x + roi.width/2;
  utm.northing = roi.y + roi.height/2;
  utm.altitude = 0.0;
  utm.zone = zone;
  utm.band = band;

  realm_msgs::GroundImageCompressed msg;
  std::vector<std::string> layer_names = map.getAllLayerNames();
  if (layer_names.size() == 1)
    msg = to_ros::groundImage(header, map[layer_names[0]], utm, map.resolution());
  else if (layer_names.size() == 2)
    msg = to_ros::groundImage(header, map[layer_names[0]], utm, map.resolution(), map[layer_names[1]]); // TODO: nobody understands that layer to is "valid"
  else
    throw(std::invalid_argument("Error publishing CvGridMap: More than one layer provided!"));

  publisher.publish(msg);
}

void StageNode::pubTrajectory(const std::vector<geometry_msgs::PoseStamped> &traj, const std::string &topic)
{
  ros::Publisher publisher = _publisher[topic];
  if (publisher.getNumSubscribers() == 0)
    return;

  nav_msgs::Path msg;
  msg.header = traj.back().header;
  msg.poses = traj;
  publisher.publish(msg);
}

bool StageNode::srvFinish(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("STAGE_NODE [%s]: Requesting stage finishCallback...!", _type_stage.c_str());
  _stage->requestFinish();
  _stage->join();
  res.success = 1;
  res.message = "Successfully finished stage!";
  ROS_INFO("STAGE_NODE [%s]: Successfully finished stage!", _type_stage.c_str());
  ROS_INFO("STAGE_NODE [%s]: Shutting stage node down...", _type_stage.c_str());
  std::unique_lock<std::mutex> lock(_mutex_do_shutdown);
  _do_shutdown = true;
  return true;
}

bool StageNode::srvStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("STAGE_NODE [%s]: Requesting stage stop...!", _type_stage.c_str());
  _stage->requestStop();
  res.success = 1;
  res.message = "Successfully stopped stage!";
  ROS_INFO("STAGE_NODE [%s]: Successfully stopped stage!", _type_stage.c_str());
  return true;
}

bool StageNode::srvResume(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("STAGE_NODE [%s]: Requesting stage resume...!", _type_stage.c_str());
  _stage->resume();
  res.success = 1;
  res.message = "Successfully resumed stage!";
  ROS_INFO("STAGE_NODE [%s]: Successfully resumed stage!", _type_stage.c_str());
  return true;
}

bool StageNode::srvReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("STAGE_NODE [%s]: Requesting stage reset...!", _type_stage.c_str());
  _stage->requestReset();
  res.success = 1;
  res.message = "Successfully reset stage!";
  ROS_INFO("STAGE_NODE [%s]: Successfully reset stage!", _type_stage.c_str());
  return true;
}

bool StageNode::srvChangeParam(realm_msgs::ParameterChange::Request &req, realm_msgs::ParameterChange::Response &res)
{
  ROS_INFO("STAGE_NODE [%s]: Changing stage parameter %s to value %s...", _type_stage.c_str(), req.name.c_str(), req.val.c_str());
  if (_stage->changeParam(req.name, req.val))
  {
    ROS_INFO("STAGE_NODE [%s]: Successfully changed parameter!", _type_stage.c_str());
    res.success = 1;
    res.message = "Successfully changed parameter!";
    return true;
  }
  else
  {
    ROS_INFO("STAGE_NODE [%s]: Failed to change parameter!", _type_stage.c_str());
    res.success = 0;
    res.message = "Failed to change parameter!";
    return false;
  }
}

void StageNode::readStageSettings()
{
  // Load stage settings
  ROS_INFO("STAGE_NODE [%s]: Loading stage settings from path:\n\t%s", _type_stage.c_str(), _file_settings_stage.c_str());
  _settings_stage = StageSettingsFactory::load(_type_stage, _file_settings_stage);
  ROS_INFO("STAGE_NODE [%s]: Detected stage type: '%s'", _type_stage.c_str(), (*_settings_stage)["type"].toString().c_str());
}

void StageNode::readParams()
{
  // Read parameters from launch file
  ros::NodeHandle param_nh("~");
  param_nh.param("stage/type", _type_stage, std::string("uninitialised"));
  param_nh.param("stage/master", _is_master_stage, false);
  param_nh.param("stage/output_dir", _path_output, std::string("uninitialised"));
  param_nh.param("topics/input", _topic_frame_in, std::string("uninitialised"));
  param_nh.param("topics/output", _topic_frame_out, std::string("uninitialised"));
  param_nh.param("config/id", _id_camera, std::string("uninitialised"));
  param_nh.param("config/profile", _profile, std::string("uninitialised"));
  param_nh.param("config/method", _method, std::string("uninitialised"));

  // Set specific config file paths
  if (_profile == "uninitialised")
    throw(std::invalid_argument("Error: Stage settings profile must be provided in launch file."));
}

void StageNode::setPaths()
{
  // Set config path. It is currently fixed to .../realm_ros/config/...
  _path_package = ros::package::getPath("realm_ros");
  _path_profile = _path_package + "/profiles/" + _profile;
  _path_output = _path_package + "/output";

  // Set settings filepaths
  _file_settings_camera = _path_profile + "/camera/calib.yaml";
  _file_settings_stage = _path_profile + "/" + _type_stage + "/stage_settings.yaml";
  _file_settings_method = _path_profile + "/" + _type_stage + "/method/" + _method + "_settings.yaml";

  if (!io::dirExists(_path_profile))
    throw(std::runtime_error("Error: Profile folder '" + _path_profile + "' was not found!"));
  if (!io::dirExists(_path_output))
    io::createDir(_path_output);

  // Master priviliges
  if (_is_master_stage)
  {
    // Create sub directory with timestamp
    _dir_output = io::getDateTime();
    if (!io::dirExists(_path_output + "/" + _dir_output))
      io::createDir(_path_output + "/" + _dir_output);
  }
}

void StageNode::setTfBaseFrame(const UTMPose &utm)
{
  tf::Vector3 origin(utm.easting, utm.northing, 0.0);
  tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
  _tf_base = tf::Transform(q, origin);
  _is_tf_base_initialized = true;

  geographic_msgs::GeoPoint wgs = to_ros::wgs84(utm);
  _gnss_base.latitude = wgs.latitude;
  _gnss_base.longitude = wgs.longitude;
  _gnss_base.altitude = wgs.altitude;

  ROS_INFO("STAGE_NODE [%s]: Frame reference set at: %f, %f", _type_stage.c_str(), _gnss_base.latitude, _gnss_base.longitude);
}