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

#ifndef PROJECT_POSE_ESTIMATION_STAGE_H
#define PROJECT_POSE_ESTIMATION_STAGE_H

#include <iostream>

#include <realm_stages/stage_base.h>
#include <realm_stages/stage_settings.h>
#include <realm_core/frame.h>
#include <realm_core/camera_settings.h>
#include <realm_core/structs.h>
#include <realm_io/cv_export.h>
#include <realm_io/realm_export.h>
#include <realm_io/exif_export.h>
#include <realm_vslam_base/dummy_referencer.h>
#include <realm_vslam_base/geometric_referencer.h>
#include <realm_vslam_base/visual_slam_factory.h>

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
    };

  public:
    PoseEstimation(const StageSettings::Ptr &stage_set,
                   const VisualSlamSettings::Ptr &vslam_set,
                   const CameraSettings::Ptr &cam_set,
                   double rate);
    ~PoseEstimation();
    void addFrame(const Frame::Ptr &frame) override;
    bool process() override;
  private:
    // Flag for initialization of georeference
    bool _is_georef_initialized;

    // Flag to disable usage of visual slam. If Flag is set to 'false', then this stage
    // only works as image throttle according to the max overlap defined in the settings
    bool _use_vslam;

    // Flag to disable fallback solution based on lat/lon/alt/heading completely
    bool _use_fallback;

    // Flag to disable using an initial guess of the camera pose to make tracking more stable in the visual SLAM
    bool _use_initial_guess;

    // Flag to disable georeferencing updates after initialization. Might result in a more consistent map, but worse
    // georeferencing results
    bool _do_update_georef;

    // Flag to suppress publish of outdated poses after initialization of georeference. Might be needed if visual pose
    // is fed to state estimation filters
    bool _do_suppress_outdated_pose_pub;

    // Threshold for error of georeference before initializing
    double _th_error_georef;

    // settings
    FallbackStrategy _strategy_fallback;
    double _overlap_max;          // [%] Maxmimum overlap for every publish to be checked, even keyframes
    double _overlap_max_fallback; // [%] Maximum overlap for fallback publishes, e.g. GNSS only

    SaveSettings _settings_save;

    // Transformation from visual world to geo coordinate frame
    std::mutex _mutex_t_w2g;
    cv::Mat _T_w2g;

    // Current debug image, gets published by PoseEstimationIO
    // Warning: As soon as published, it will get released
    // Always lock mutex when reading and check for empty()
    std::mutex _mutex_img_debug;
    cv::Mat _img_debug;

    // Overlap estimation
    Plane _plane_ref;       // Reference plane for projection, normally (0,0,0), (0,0,1)
    cv::Rect2d _roi_prev;   // Previous published roi, used for overlap calculation

    // Buffer for all frames added
    std::deque<Frame::Ptr> _buffer_no_pose;
    std::deque<Frame::Ptr> _buffer_pose_all;
    std::vector<Frame::Ptr> _buffer_pose_init;
    std::deque<Frame::Ptr> _buffer_do_publish;  // Note: Will only get cleared by user reset
    // Thread safety
    std::mutex _mutex_buffer_no_pose;
    std::mutex _mutex_buffer_pose_all;
    std::mutex _mutex_buffer_pose_init;
    std::mutex _mutex_buffer_do_publish;

    // handles
    std::mutex _mutex_vslam;
    VisualSlamIF::Ptr _vslam;

    // Publisher thread
    std::unique_ptr<PoseEstimationIO> _stage_publisher;

    // Georeferencing initializer
    GeospatialReferencerIF::Ptr _georeferencer;

    void track(Frame::Ptr &frame);

    void evaluateFallbackStrategy(FallbackStrategy strategy);

    void reset() override;
    void initStageCallback() override;
    void printSettingsToLog() override;

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
    cv::Rect2d estimateProjectedRoi(const Frame::Ptr &frame);
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
    void initLog(const std::string &filepath);

  private:
    bool _is_time_ref_set;
    bool _do_delay_keyframes;
    bool _is_new_output_path_set;

    std::string _path_output;

    PoseEstimation* _stage_handle;

    std::mutex _mutex_schedule;
    TimeReference _t_ref;
    std::deque<Task> _schedule;

    // online processing
    void reset() override;
    void publishPose(const Frame::Ptr &frame);
    void publishSurfacePoints(const Frame::Ptr &frame);
    void publishFrame(const Frame::Ptr &frame);
    void scheduleFrame(const Frame::Ptr &frame);
    void publishScheduled();
};

} // namespace stages
} // namespace realm

#endif //PROJECT_POSE_ESTIMATION_STAGE_H
