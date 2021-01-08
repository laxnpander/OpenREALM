

#ifndef PROJECT_PLANE_SWEEP_H
#define PROJECT_PLANE_SWEEP_H

#include <fstream>
#include <iostream>
#include <cstdint>

#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <psl/cameraMatrix.h>
#include <psl/cudaPlaneSweep.h>

#include <realm_core/frame.h>
#include <realm_densifier_base/densifier_IF.h>
#include <realm_densifier_base/densifier_settings.h>

namespace realm
{
namespace densifier
{

class PlaneSweep : public DensifierIF
{
  public:
    struct Settings
    {
        bool enable_subpix;
        bool enable_color_match;
        bool enable_out_best_depth;
        bool enable_out_best_cost;
        bool enable_out_cost_vol;
        bool enable_out_uniq_ratio;
        int nrof_planes;
        double scale;
        cv::Size match_window_size;
        PSL::PlaneSweepOcclusionMode occlusion_mode;
        PSL::PlaneSweepPlaneGenerationMode plane_gen_mode;
        PSL::PlaneSweepMatchingCosts match_cost;
        PSL::PlaneSweepSubPixelInterpMode subpx_interp_mode;
    };
  public:

    /*!
     * @brief Constructor for plane sweep interface class. Pretty much straightforward. Settings are read and set.
     * @param settings Densifier settings is the base class of the passed settings. For derived, concrete settings
     *        see "densifier_settings.h". Contains all necessary information to instantiate a plane sweep.
     */
    explicit PlaneSweep(const DensifierSettings::Ptr &settings);

    /*!
     * @brief Essential function for densification of input frame vector based on the plane sweep library. For reference
     *        idx the dense depth map will be calculated and returned as CV_32F opencv matrix. Depth values are not
     *        encoded, but directly written into the pixel positions.
     * @param frames Vector of frames to be densified. Must have at least two frames.
     * @param ref_idx Idx of reference frame inside the "@param frames vector".
     * @return Densified depth map of the observed scene for reference frame frames[ref_idx]
     */
    Depthmap::Ptr densify(const std::deque<Frame::Ptr> &frames, uint8_t ref_idx) override;

    /*!
     * @brief Getter for number of input frames for densification
     * @return Number of frames that "densify" needs inside the frame vector
     */
    uint8_t getNrofInputFrames() override;

    /*!
     * @brief Getter for the resize factor of the densified images. Is a value between 0.0 < resize factor <= 1.0
     * @return Resize factor
     */
    double getResizeFactor() override;

    /*!
     * @brief Prints all settings to the log file for documentation.
     */
    void printSettingsToLog() override;

  private:

    //! Number of frames for SFM
    uint8_t m_nrof_frames;

    //! Resize factor for input images
    double m_resizing;

    //! Struct of the plane sweep settings
    Settings m_settings;

    /*!
     * @brief Function to fix the input image type to a PSL conform type. Depends on the arguments se (use rgb or not)
     * @param img Input image to be converted
     * @return Converted image ready to be passed to PSL
     */
    cv::Mat fixImageType(const cv::Mat &img);

    /*!
     * @brief Converter for PSL library depth map to OpenCV matrix type
     * @param depthmap PSL depth map type
     */
    cv::Mat convertToCvMat(PSL::DepthMap<float, double> &depthmap);

    /*!
     * @brief Converter for REALM pinhole camera to PSL camera type
     * @param cam REALM pinhole camera
     * @return PSL pinhole camera
     */
    PSL::CameraMatrix<double> convertToPslCamera(const camera::Pinhole::Ptr &cam);
};

} // namespace densifier
} // namespace realm

#endif //PROJECT_PLANE_SWEEP_H
