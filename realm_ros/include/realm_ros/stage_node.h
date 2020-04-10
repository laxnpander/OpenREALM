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

#ifndef PROJECT_STAGE_NODE_H
#define PROJECT_STAGE_NODE_H

#include <mutex>
#include <unordered_map>
#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>

#include <realm_core/structs.h>
#include <realm_core/camera_settings_factory.h>
#include <realm_vslam_base/visual_slam_settings_factory.h>
#include <realm_densifier_base/densifier_settings_factory.h>
#include <realm_io/utilities.h>
#include <realm_ros/conversions.h>

#include <realm_stages/stage_settings_factory.h>
#include <realm_stages/pose_estimation.h>
#include <realm_stages/densification.h>
#include <realm_stages/surface_generation.h>
#include <realm_stages/ortho_rectification.h>
#include <realm_stages/mosaicing.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <realm_msgs/Frame.h>
#include <realm_msgs/CvGridMap.h>
#include <realm_msgs/GroundImageCompressed.h>

#include <std_srvs/Trigger.h>
#include <realm_msgs/ParameterChange.h>

namespace realm
{

class StageNode
{
  public:
    StageNode(int argc, char **argv);
    ~StageNode();
    void spin();
    bool isOkay();
  private:
    // Master stage has several privileges,
    // e.g. creating output folder, ...
    bool _is_master_stage;

    // Set to true, to shut node down
    std::mutex _mutex_do_shutdown;
    bool _do_shutdown;

    // number of msgs that triggere ros CB
    uint32_t _nrof_msgs_rcvd;

    // type of stage
    std::string _type_stage;
    std::string _type_method;

    // camera info
    std::string _id_camera;

    // ros handle
    ros::NodeHandle _nh;

    // ros communication handles
    ros::Subscriber _sub_input_frame;
    ros::Subscriber _sub_output_dir;
    std::unordered_map<std::string, ros::Publisher> _publisher;

    // ros service handles
    ros::ServiceServer _srv_req_finish;
    ros::ServiceServer _srv_req_stop;
    ros::ServiceServer _srv_req_resume;
    ros::ServiceServer _srv_req_reset;
    ros::ServiceServer _srv_change_param;

    // chosen profile related to all settings set
    std::string _profile;

    // chosen method related to the specific framework implementation
    std::string _method;

    // working paths
    std::string _path_package;
    std::string _path_profile;
    std::string _path_output;

    // working directories
    std::string _dir_output;

    // filename of settings
    std::string _file_settings_stage;
    std::string _file_settings_method;
    std::string _file_settings_camera;

    // topics
    std::string _topic_prefix;
    std::string _topic_frame_in;
    std::string _topic_frame_out;

    // transforms
    bool _is_tf_base_initialized;
    bool _is_tf_stage_initialized;
    std::string _tf_base_frame_name;
    std::string _tf_stage_frame_name;
    tf::Transform _tf_base;
    tf::Transform _tf_stage;
    sensor_msgs::NavSatFix _gnss_base;

    // trajectories
    std::unordered_map<std::string, std::vector<geometry_msgs::PoseStamped>> _trajectories;

    // Settings of the stage
    StageSettings::Ptr _settings_stage;

    // Handle for stage
    StageBase::Ptr _stage;

    // Initialization
    void readParams();
    void readStageSettings();
    void setPaths();
    void setTfBaseFrame(const UTMPose &utm);
    void createStagePoseEstimation();
    void createStageDensification();
    void createStageSurfaceGeneration();
    void createStageOrthoRectification();
    void createStageMosaicing();
    void linkStageTransport();

    // Functionalities
    void reset();

    // ros communication functions
    void subFrame(const realm_msgs::Frame &msg);
    void subOutputPath(const std_msgs::String &msg);

    // stage callbacks
    void pubFrame(const Frame::Ptr &frame, const std::string &topic);
    void pubPose(const cv::Mat &pose, uint8_t zone, char band, const std::string &topic);
    void pubPointCloud(const cv::Mat &pts, const std::string &topic);
    void pubImage(const cv::Mat &img, const std::string &topic);
    void pubDepthMap(const cv::Mat &img, const std::string &topic);
    void pubMesh(const std::vector<Face> &faces, const std::string &topic);

    // master publish
    void pubTrajectory(const std::vector<geometry_msgs::PoseStamped> &traj, const std::string &topic);

    /*!
     * @brief Publisher for CvGridMaps as GroundImages. Should either contain one or two layers. The first layer
     *        represents the visual informations and the second (optional) the mask or valid elements of the visual info.
     * @param map Either 1- or 2-layer grid map containing visual informations (and valid elements of visual img)
     * @param zone UTM zone of the published CvGridMap
     * @param band UTM band of the published CvGridMap
     * @param topic Topic is NOT the ros topic, but the topic for REALM to identify which publisher should be triggered.
     */
    void pubCvGridMap(const CvGridMap &map, uint8_t zone, char band, const std::string &topic);

    bool srvFinish(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool srvStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool srvResume(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool srvReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool srvChangeParam(realm_msgs::ParameterChange::Request &req, realm_msgs::ParameterChange::Response &res);
};

} // namespace realm

#endif //PROJECT_STAGE_NODE_H
