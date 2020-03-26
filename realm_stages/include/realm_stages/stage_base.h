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

#ifndef PROJECT_STAGE_H
#define PROJECT_STAGE_H

#include <iostream>
#include <functional>
#include <thread>
#include <chrono>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>

#include <realm_types/frame.h>
#include <realm_common/timer.h>
#include <realm_types/structs.h>
#include <realm_types/worker_thread_base.h>
#include <realm_types/settings_base.h>

namespace realm
{

/*!
 * @brief Similiar to state of the art photogrammetry software REALM is designed as pipeline. The idea behind the
 * pipeline architecture is to break down the complex problem into several independent steps. Similiar to a conveyor
 * images are transported from one step (stage) to another doing all the possible processing work and then moving on.
 * One basic example might be: ImageCapturing -> PoseEstimation -> Densification -> OrthoGeneration -> Visualization
 * These 5 stages can and should be processed in a seperate thread each, which are also independent from the
 * communication infrastructure (e.g. ROS). So if we have 5 steps in the pipeline, each step should have one thread
 * that does the communication work and feeds the data into the second thread which does the processing. This class
 * provides and abstract for the latter one. Each stage should implement the stage class, which in turn takes care
 * of thread handling like start, stop, reset without conflicting with the communication thread feeding data.
 */

class StageBase : public WorkerThreadBase
{
  public:
    using Ptr = std::shared_ptr<StageBase>;
    using ConstPtr = std::shared_ptr<const StageBase>;

    using FrameTransportFunc = std::function<void(const realm::Frame::Ptr &, const std::string &)>;
    using PoseTransportFunc = std::function<void(const cv::Mat &, uint8_t zone, char band, const std::string &)>;
    using DepthMapTransportFunc = std::function<void(const cv::Mat &, const std::string &)>;
    using PointCloudTransportFunc = std::function<void(const cv::Mat &, const std::string &)>;
    using ImageTransportFunc = std::function<void(const cv::Mat &, const std::string &)>;
    using MeshTransportFunc = std::function<void(const std::vector<Face> &, const std::string &)>;
    using CvGridMapTransportFunc = std::function<void(const CvGridMap &, uint8_t zone, char band, const std::string &)>;
  public:
    /*!
     * @brief Basic constructor for stage class
     * @param name Name of the stage, should be set by derived stage
     * @param path Path to the input/output folder for stage informations
     */
    StageBase(const std::string &name, const std::string &path, int queue_size);

    /*!
     * @brief Communication thread of the stage will receive data from the previous stage and feed it into the
     * processing thread through this function. The data structure and implementation itself can be chosen by the
     * derived stage.
     * @param frame Frame that contains all neccessary data for the derived processing stage
     */
    virtual void addFrame(const Frame::Ptr &frame) = 0;

    /*!
     * @brief Function that allows to change parameters in derived stage on the fly. However, only parameter that were
     *        implemented can be changed. If no change is desired, don't implement this function.
     * @param name Name of the parameter, should at best match it's name in stage_settings
     * @param val Value of the parameter, should be parsed in implementation.
     */
    virtual bool changeParam(const std::string &name, const std::string &val);

    /*!
     * @brief Function to print the most essential settings to the current log file. Must be implemented by the derived
     *        stage and will be called after stage path is known.
     */
    virtual void printSettingsToLog() = 0;

    /*!
     * @brief Function to set the output path for data acquisition. Mostly for debugging purposes. Also triggers
     * "createOutputDirectories" (see below)
     * @param abs_path absolut path to the folder, in which subfolders of output data will be created by derived stage
     */
    void initStagePath(const std::string &abs_path);

    /*!
     * @brief Because REALM is independent from the communication infrastructure (e.g. ROS), a transport to the
     * corresponding communication interface has to be defined. We chose to use callback functions, that can be
     * triggered inside the derived stage to transport results. Therefore the callbacks MUST be set, otherwise no data
     * will leave the stage.
     * @param func This function consists of a result frame, a defined topic as description for the data (for example:
     * "output/result_frame". Timestamp may or may not be set inside the stage     */
    void registerFrameTransport(const FrameTransportFunc &func);

    /*!
     * @brief Because REALM is independent from the communication infrastructure (e.g. ROS), a transport to the
     * corresponding communication interface has to be defined. We chose to use callback functions, that can be
     * triggered inside the derived stage to transport results. Therefore the callbacks MUST be set, otherwise no data
     * will leave the stage.
     * @param func This function consists of a result pose, a defined topic as description for the data (for example:
     * "output/result_pose". Timestamp may or may not be set inside the stage     */
    void registerPoseTransport(const PoseTransportFunc &func);

    /*!
     * @brief Because REALM is independent from the communication infrastructure (e.g. ROS), a transport to the
     * corresponding communication interface has to be defined. We chose to use callback functions, that can be
     * triggered inside the derived stage to transport results. Therefore the callbacks MUST be set, otherwise no data
     * will leave the stage.
     * @param func This function consists of a result pointcloud as cv::Mat with row() = (x,y,z,r,g,b,nx,ny,nz), a
     * defined topic as description for the data (for example: "output/result_pose".       */
    void registerPointCloudTransport(const PointCloudTransportFunc &func);

    /*!
     * @brief Because REALM is independent from the communication infrastructure (e.g. ROS), a transport to the
     * corresponding communication interface has to be defined. We chose to use callback functions, that can be
     * triggered inside the derived stage to transport results. Therefore the callbacks MUST be set, otherwise no data
     * will leave the stage.
     * @param func This function consists of a result depth map as cv::Mat of type CV_32F and a defined topic as
     * description for the data (for example: "output/depth".       */
    void registerDepthMapTransport(const DepthMapTransportFunc &func);

    /*!
     * @brief Because REALM is independent from the communication infrastructure (e.g. ROS), a transport to the
     * corresponding communication interface has to be defined. We chose to use callback functions, that can be
     * triggered inside the derived stage to transport results. Therefore the callbacks MUST be set, otherwise no data
     * will leave the stage.
     * @param func This function consists of a result img, a defined topic as description for the data (for example:
     * "output/result_img". Timestamp may or may not be set inside the stag      */
    void registerImageTransport(const ImageTransportFunc &func);

    /*!
     * @brief Because REALM is independent from the communication infrastructure (e.g. ROS), a transport to the
     * corresponding communication interface has to be defined. We chose to use callback functions, that can be
     * triggered inside the derived stage to transport results. Therefore the callbacks MUST be set, otherwise no data
     * will leave the stage.
     * @param func This function consists of a vector of faces, a defined topic as description for the data (for example:
     * "output/result_frame". Timestamp may or may not be set inside the stage     */
    void registerMeshTransport(const MeshTransportFunc &func);

    /*!
     * @brief Because REALM is independent from the communication infrastructure (e.g. ROS), a transport to the
     * corresponding communication interface has to be defined. We chose to use callback functions, that can be
     * triggered inside the derived stage to transport results. Therefore the callbacks MUST be set, otherwise no data
     * will leave the stage.
     * @param func This function consists of a CvGridMap type, a defined topic as description for the data (for example:
     * "output/result_gridmap". Timestamp may or may not be set inside the stage fo  */
    void registerCvGridMapTransport(const CvGridMapTransportFunc &func);

  protected:

    bool _is_output_dir_initialized;

    /*!
     * @brief The counter are used to compute the incoming and outgoing frame frequency. They will be evaluated and
     * reseted by the timer every X seconds.
     */
    uint32_t _counter_frames_in;
    uint32_t _counter_frames_out;
    uint32_t _t_statistics_period; // [seconds]
    Timer::Ptr _timer_statistics_fps;
    std::mutex _mutex_statistics_fps;

    /*!
     * @brief Queue size of the added frames. Usually implemented as ringbuffer / fifo
     */
    int _queue_size;

    /*!
     * @brief Short name of the implemented stage. Should be written by derived classes
     */
    std::string _stage_name;

    /*!
     * @brief Path of the stage package. Is used for output writing.
     */
    std::string _stage_path;

    /*!
     * @brief This function consists of a result frame, a defined topic as description for the data (for example:
     * "output/result_frame". ll be set through "registerFrameTransport".
     */
    FrameTransportFunc _transport_frame;

    /*!
     * @brief This function consists of a result pose, a defined topic as description for the data (for example:
     * "output/result_pose". one and band can also be set, to make sure the x/y are interpreted
     * correctly (normally utm coordinates). Will be set through "registerPoseTransport".
     */
    PoseTransportFunc _transport_pose;

    /*!
     * @brief This function consists of a result pointcloud as cv::Mat with (x_i, y_i, z_i), a defined topic as
     * description for the data (for example: "output/result_pcl". Will be set through "registerFrameTransport".
     */
    PointCloudTransportFunc _transport_pointcloud;

    /*!
     * @brief This function consists of a result depth map as cv::Mat of type CV_32F and a defined topic as
     * description for the data (for example: "output/depth"). Will be set through "registerFrameTransport".
     */
    DepthMapTransportFunc _transport_depth_map;

    /*!
     * @brief This function consists of a result img, a defined topic as description for the data (for example:
     * "output/result_img". Will be set through "registerFrameTransport".
     */
    ImageTransportFunc _transport_img;

    /*!
     * @brief This function consists of a vector of faces, a defined topic as description for the data (for example:
     * "output/result_frame". ll be set through "registerMeshTransport".
     */
    MeshTransportFunc _transport_mesh;

    /*!
     * @brief This function consists of a CvGridMap, a defined topic as description for the data (for example:
     * "output/result_gridmap".  be set through "registerCvGridMapTransport".
     */
    CvGridMapTransportFunc _transport_cvgridmap;

    /*!
     * @brief Function for creation of all neccessary output directories of the derived stage. Will be called whenever
     * "initStagePath" was triggered.
     */
    virtual void initStageCallback() = 0;

    /*!
     * @brief Setter for the statistics evaluation period.
     * @param s Period of time in seconds
     */
    void setStatisticsPeriod(uint32_t s);

    /*!
     * @brief Update function to be called by the derived class to update the incoming frame rate statistic.
     */
    void updateFpsStatisticsIncoming();

    /*!
     * @brief Update function to be called by the derived class to update the outgoing frame rate statistic.
     */
    void updateFpsStatisticsOutgoing();

    /*
     * brief Gets triggered by _timer_statistics_fps every X seconds to read the incoming and outgoing number of frame
     * counters.
     */
    void evaluateFpsStatistic();
};

} // namespace realm

#endif //PROJECT_STAGE_H
