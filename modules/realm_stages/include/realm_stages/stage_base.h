

#ifndef PROJECT_STAGE_H
#define PROJECT_STAGE_H

#include <iostream>
#include <functional>
#include <thread>
#include <chrono>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>

#include <realm_core/frame.h>
#include <realm_core/timer.h>
#include <realm_core/structs.h>
#include <realm_core/worker_thread_base.h>
#include <realm_core/settings_base.h>

namespace realm
{

/*!
 * @brief Structure to hold performance statistics for a stage.  This is mostly concerned with the state of the
 * frame queue, processing time, and any overruns.
 */
struct StageStatistics {
  float fps_in{};
  float fps_out{};
  uint32_t queue_depth{};
  uint32_t frames_total{};
  uint32_t frames_dropped{};
  uint32_t frames_bad{};
  uint32_t frames_processed{};
  Statistics queue_statistics{};
  Statistics process_statistics{};
};

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
    using PointCloudTransportFunc = std::function<void(const PointCloud::Ptr &, const std::string &)>;
    using ImageTransportFunc = std::function<void(const cv::Mat &, const std::string &)>;
    using MeshTransportFunc = std::function<void(const std::vector<Face> &, const std::string &)>;
    using CvGridMapTransportFunc = std::function<void(const CvGridMap &, uint8_t zone, char band, const std::string &)>;
  public:
    /*!
     * @brief Basic constructor for stage class
     * @param name Name of the stage, should be set by derived stage
     * @param path Path to the input/output folder for stage informations
     * @param rate The rate to run the stage at
     * @param queue_size The maximum depth of the main queue for the stage
     * @param log_to_file True to log message to the stage directory, false to log to stdout/stderr only
     */
    StageBase(const std::string &name, const std::string &path, double rate, int queue_size, bool log_to_file);

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
     * @brief Gets statistics on frame rates, queue sizes, processing times, and other useful information about the
     * runtime status of the stage. For this to have data, the correct calls to update statistics must be made
     * in the implemented module.
     * @return A structure with the current stage statistics.
     */
    StageStatistics getStageStatistics();


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

    bool m_is_output_dir_initialized;

    /*!
     * @brief The counter are used to compute the incoming and outgoing frame frequency. They will be evaluated and
     * reseted by the timer every X seconds.
     */
    uint32_t m_counter_frames_in;
    uint32_t m_counter_frames_out;
    uint32_t m_t_statistics_period; // [seconds]
    Timer::Ptr m_timer_statistics_fps;

    /*!
     * @brief Queue size of the added frames. Usually implemented as ringbuffer / fifo
     */
    int m_queue_size;

    /*!
     * @brief Statistics for the stage including processing time, queue depths, and other useful information
     */
    StageStatistics m_stage_statistics{};
    mutable std::mutex m_mutex_statistics;

    /*!
     * @brief Short name of the implemented stage. Should be written by derived classes
     */
    std::string m_stage_name;

    /*!
     * @brief Path of the stage package. Is used for output writing.
     */
    std::string m_stage_path;

    /*!
     * @brief flag to indicate if we should write to a separate log file or not
     */
    bool m_log_to_file;

    /*!
     * @brief This function consists of a result frame, a defined topic as description for the data (for example:
     * "output/result_frame". ll be set through "registerFrameTransport".
     */
    FrameTransportFunc m_transport_frame;

    /*!
     * @brief This function consists of a result pose, a defined topic as description for the data (for example:
     * "output/result_pose". one and band can also be set, to make sure the x/y are interpreted
     * correctly (normally utm coordinates). Will be set through "registerPoseTransport".
     */
    PoseTransportFunc m_transport_pose;

    /*!
     * @brief This function consists of a result pointcloud as cv::Mat with (x_i, y_i, z_i), a defined topic as
     * description for the data (for example: "output/result_pcl". Will be set through "registerFrameTransport".
     */
    PointCloudTransportFunc m_transport_pointcloud;

    /*!
     * @brief This function consists of a result depth map as cv::Mat of type CV_32F and a defined topic as
     * description for the data (for example: "output/depth"). Will be set through "registerFrameTransport".
     */
    DepthMapTransportFunc m_transport_depth_map;

    /*!
     * @brief This function consists of a result img, a defined topic as description for the data (for example:
     * "output/result_img". Will be set through "registerFrameTransport".
     */
    ImageTransportFunc m_transport_img;

    /*!
     * @brief This function consists of a vector of faces, a defined topic as description for the data (for example:
     * "output/result_frame". ll be set through "registerMeshTransport".
     */
    MeshTransportFunc m_transport_mesh;

    /*!
     * @brief This function consists of a CvGridMap, a defined topic as description for the data (for example:
     * "output/result_gridmap".  be set through "registerCvGridMapTransport".
     */
    CvGridMapTransportFunc m_transport_cvgridmap;

    /*!
     * @brief Setting an async data ready functor allows the thread to wake up from sleep outside the sleep time. It
     * will only sleep as long as the data ready functor returns falls. It  could therefore be provided with a function
     * that checks the size of a processing queue and returns true as long as there is data inside.
     * @param func Functor that tells the worker thread when there is something to process and when there is not.
     */
    void registerAsyncDataReadyFunctor(const std::function<bool()> &func);

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
     * @brief Function can be called to log the current statistics. Shouldn't be called on every processed frame to not
     * pollute the log.
     */
    void logCurrentStatistics() const;

    /*!
    @brief Gets the current stage queue depth
    */
    virtual uint32_t getQueueDepth() = 0;

    /*!
     * @brief Update function to be called by the derived class to update the incoming statistics
     */
    void updateStatisticsIncoming();

    /*!
     * @brief Update function to be called by the derived class to update statistics when a frame is skipped due to
     * the queue filling up.
     */
    void updateStatisticsSkippedFrame();

    /*!
     * @brief Update function to be called by the derived class to update the incoming frame rate statistic.
     */
    void updateStatisticsBadFrame();

    /*!
     * @brief Update function to be called by derived class to update number of processed frames in the statistic.
     * Information might be redundant with total and dropped frames.
     */
    void updateStatisticsProcessedFrame();

    /*!
     * @brief Update function to be called by the derived class to update the outgoing frame rate statistic.
     */
    void updateStatisticsOutgoing();

    /*
     * brief Gets triggered by _timer_statistics_fps every X seconds to read the incoming and outgoing number of frame
     * counters.
     */
    void evaluateStatistic();
};

} // namespace realm

#endif //PROJECT_STAGE_H
