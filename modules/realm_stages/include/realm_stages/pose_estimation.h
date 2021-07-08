

#ifndef PROJECT_POSE_ESTIMATION_STAGE_H
#define PROJECT_POSE_ESTIMATION_STAGE_H

#include <iostream>

#include <realm_stages/stage_base.h>
#include <realm_stages/stage_settings.h>
#include <realm_core/frame.h>
#include <realm_core/camera_settings.h>
#include <realm_core/imu_settings.h>
#include <realm_core/structs.h>
#include <realm_io/cv_export.h>
#include <realm_io/realm_export.h>
#include <realm_vslam_base/dummy_referencer.h>
#include <realm_vslam_base/geometric_referencer.h>
#include <realm_vslam_base/visual_slam_factory.h>

#ifdef WITH_EXIV2
#include <realm_io/exif_export.h>
#endif

namespace realm
{
namespace stages
{

class PoseEstimationIO;

class PoseEstimation : public StageBase
{
  public:
    using Ptr = std::shared_ptr<PoseEstimation>;
    using ConstPtr = std::shared_ptr<const PoseEstimation>;
    friend PoseEstimationIO;

    /*!
     * @brief Fallback strategies ensure, that even when tracking is lost, the mapping process can continue. This is realized
     * by using the UAV's GPS coordinates and assume a default camera pose can be estimated, e.g.
     * - based on magnetic heading and a downward pointing camera
     * - based on IMU readings provided with each frame
     */
    enum class FallbackStrategy
    {
      ALWAYS,               // Use this strategy, only, when provided orientation in the frame aligns with the camera orientation
      NEVER                 // Use this strategy, when you don't want fallback projection based on GPS coordinates only
    };

    struct SaveSettings
    {
        bool save_trajectory_gnss;
        bool save_trajectory_visual;
        bool save_frames;
        bool save_keyframes;
        bool save_keyframes_full;

        bool save_required()
        {
          return save_trajectory_gnss || save_trajectory_visual ||
                 save_frames || save_keyframes || save_keyframes_full;
        }

        bool save_trajectory()
        {
          return save_trajectory_gnss || save_trajectory_visual;
        }
    };

  public:
    PoseEstimation(const StageSettings::Ptr &stage_set,
                   const VisualSlamSettings::Ptr &vslam_set,
                   const CameraSettings::Ptr &cam_set,
                   const ImuSettings::Ptr &imu_set,
                   double rate);
    ~PoseEstimation();

    void finishCallback() override;
    void addFrame(const Frame::Ptr &frame) override;
    bool process() override;

    void queueImuData(const VisualSlamIF::ImuData &imu) const;

  private:
    // Flag for initialization of georeference
    bool m_is_georef_initialized;

    // Flag to disable usage of visual slam. If Flag is set to 'false', then this stage
    // only works as image throttle according to the max overlap defined in the settings
    bool m_use_vslam;

    // Flag to enable usage of IMU. Note, that a IMU settings file must be provided.
    bool m_use_imu;

    // Flag to set all tracked frames as keyframes, consequently they are published in higher frequency for the next stage
    bool m_set_all_frames_keyframes;

    // Flag to disable fallback solution based on lat/lon/alt/heading completely
    bool m_use_fallback;

    // The maximum number of lost frames when initializing before attempting a reset
    int m_init_lost_frames_reset_count;
    int m_init_lost_frames;

    // Flag to disable using an initial guess of the camera pose to make tracking more stable in the visual SLAM
    bool m_use_initial_guess;

    // Flag to disable georeferencing updates after initialization. Might result in a more consistent map, but worse
    // georeferencing results
    bool m_do_update_georef;

    // Flag to delay keyframes by the duration it takes for the georeference to initialize. This ensure higher pose
    // and map point quality, as every frame is refined with consecutive frames as well.
    bool m_do_delay_keyframes;

    // Flag to suppress publish of outdated poses after initialization of georeference. Might be needed if visual pose
    // is fed to state estimation filters
    bool m_do_suppress_outdated_pose_pub;

    // Threshold for error of georeference before initializing
    double m_th_error_georef;
    int m_min_nrof_frames_georef;

    // Scale changes are constantly checked to identify divergence. Flag can be set to auto reset on divergence.
    bool m_do_auto_reset;
    double m_th_scale_change;

    // settings
    FallbackStrategy m_strategy_fallback;
    double m_overlap_max;          // [%] Maxmimum overlap for every publish to be checked, even keyframes
    double m_overlap_max_fallback; // [%] Maximum overlap for fallback publishes, e.g. GNSS only

    SaveSettings m_settings_save;

    // Transformation from visual world to geo coordinate frame
    std::mutex m_mutex_t_w2g;
    cv::Mat m_T_w2g;

    // Current debug image, gets published by PoseEstimationIO
    // Warning: As soon as published, it will get released
    // Always lock mutex when reading and check for empty()
    std::mutex m_mutex_img_debug;
    cv::Mat m_img_debug;

    // Overlap estimation
    Plane m_plane_ref;       // Reference plane for projection, normally (0,0,0), (0,0,1)
    cv::Rect2d m_roi_prev;   // Previous published roi, used for overlap calculation

    // Buffer for all frames added
    std::deque<Frame::Ptr> m_buffer_no_pose;
    std::deque<Frame::Ptr> m_buffer_pose_all;
    std::vector<Frame::Ptr> m_buffer_pose_init;
    std::deque<Frame::Ptr> m_buffer_do_publish;  // Note: Will only get cleared by user reset
    // Thread safety
    std::mutex m_mutex_buffer_no_pose;
    std::mutex m_mutex_buffer_pose_all;
    std::mutex m_mutex_buffer_pose_init;
    std::mutex m_mutex_buffer_do_publish;

    // handles
    std::mutex m_mutex_vslam;
    VisualSlamIF::Ptr m_vslam;

    // Publisher thread
    std::unique_ptr<PoseEstimationIO> m_stage_publisher;

    // Georeferencing initializer
    GeospatialReferencerIF::Ptr m_georeferencer;

    void track(Frame::Ptr &frame);

    void evaluateFallbackStrategy(FallbackStrategy strategy);

    void reset() override;
    void initStageCallback() override;
    void printSettingsToLog() override;
    uint32_t getQueueDepth() override;

    void applyGeoreferenceToBuffer();
    void printGeoReferenceInfo(const Frame::Ptr &frame);
    void pushToBufferNoPose(const Frame::Ptr &frame);
    void pushToBufferInit(const Frame::Ptr &frame);
    void pushToBufferAll(const Frame::Ptr &frame);
    void pushToBufferPublish(const Frame::Ptr &frame);
    void updatePreviousRoi(const Frame::Ptr &frame);
    void updateKeyframeCb(int id, const cv::Mat& pose, const cv::Mat &points);
    bool changeParam(const std::string& name, const std::string &val);
    double estimatePercOverlap(const Frame::Ptr &frame);
    Frame::Ptr getNewFrameTracking();
    Frame::Ptr getNewFramePublish();
    cv::Mat computeInitialPoseGuess(const Frame::Ptr &frame);
    void updateOrientationCorrection(const Frame::Ptr &frame);
};

class PoseEstimationIO : public WorkerThreadBase
{
  public:
    using Ptr = std::shared_ptr<PoseEstimationIO>;
    using ConstPtr = std::shared_ptr<const PoseEstimationIO>;
    using TimeReference = std::pair<long, uint64_t>;
    using Task = std::pair<long, Frame::Ptr>;
  public:
    PoseEstimationIO(PoseEstimation* stage, double rate, bool do_delay_keyframes);
    bool process() override;
    void setOutputPath(const std::string &path);

  private:
    bool m_is_time_ref_set;
    bool m_do_delay_keyframes;
    bool m_is_new_output_path_set;

    std::string m_path_output;

    PoseEstimation* m_stage_handle;

    std::mutex m_mutex_schedule;
    TimeReference m_t_ref;
    std::deque<Task> m_schedule;

    // online processing
    void reset() override;
    void publishPose(const Frame::Ptr &frame);
    void publishSparseCloud(const Frame::Ptr &frame);
    void publishFrame(const Frame::Ptr &frame);
    void scheduleFrame(const Frame::Ptr &frame);
    void publishScheduled();
    void publishAll();
};

} // namespace stages
} // namespace realm

#endif //PROJECT_POSE_ESTIMATION_STAGE_H
