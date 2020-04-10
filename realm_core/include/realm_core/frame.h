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

#ifndef PROJECT_FRAME_H
#define PROJECT_FRAME_H

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <realm_core/enums.h>
#include <realm_core/utm32.h>
#include <realm_core/camera.h>
#include <realm_core/cv_grid_map.h>

namespace realm
{

/*!
 * @brief Class to save and pass all informations, that are measured or computed at some point during the pipeline. It
 *        is the basic data container for acquired images and other measurement data. Every frame gets unique ids and
 *        timestamps, that should not be used more than once. Only the shared pointer is passed around after creation.
 */
class Frame
{
  public:
    using Ptr = std::shared_ptr<Frame>;
    using ConstPtr = std::shared_ptr<const Frame>;
  public:
    /*!
     * @brief One and only constructor for the creation of a frame.
     * @param camera_id Name of the camera it was acquired with
     * @param frame_id Unique number of the frame. Should start from zero and be incremented with the creation of every
     *        new frame
     * @param timestamp Unique timestamp of the time of acquisition
     * @param img Image data captured by a camera
     * @param utm UTM coordinates the camera that took the image (geotag)
     * @param cam Camera model that handles all projection and cv math
     */
    Frame(const std::string &camera_id,
          const uint32_t &frame_id,
          const uint64_t &timestamp,
          const cv::Mat &img,
          const UTMPose &utm,
          const camera::Pinhole &cam);

    /*!
     * @brief Deep copy constructor for frames. Use it wisely :)
     * @param f Frame to be copied
     */
    Frame(const Frame &f);

    /*!
     * @brief Getter for the camera id
     * @return Name of the camera
     */
    std::string getCameraId() const;

    /*!
     * @brief Getter for the frame id
     * @return Number of the frame
     */
    uint32_t getFrameId() const;

    /*!
     * @brief Getter for smallest scene depth of the observed surface points. Will be computed by computeSceneDepth()
     * @return Smallest depth value of the observed scene
     */
    double getMinSceneDepth() const;

    /*!
     * @brief Getter for highest scene depth of the observed surface points. Will be computed by computeSceneDepth()
     * @return Highest depth value of the observed scene
     */
    double getMaxSceneDepth() const;

    /*!
     * @brief Getter for median scene depth of the observed surface points. Will be computed by computeSceneDepth()
     * @return Median depth value of the observed scene
     */
    double getMedianSceneDepth() const;

    /*!
     * @brief Getter for a default pose, that is always computable if lat, lon, alt and heading was set in _utm
     * @return 3x4 matrix of the camera pose based on lat, lon, alt, heading
     */
    cv::Mat getDefaultPose() const;

    /*!
     * @brief Getter for the undistorted image
     * @return Undistorted image in full resolution
     */
    cv::Mat getImageUndistorted() const;

    /*!
     * @brief Getter for the raw, distorted image
     * @return Raw, distorted image in full resolution
     */
    cv::Mat getImageRaw() const;

    /*!
     * @brief Getter for the surface point cloud
     * @return Surface point cloud with row(i) = (x, y, z)
     */
    cv::Mat getSurfacePoints() const;

    /*!
     * @brief Getter for frame pose. Is always up to date, therefore either the default, visual or georeferenced pose
     *        If you want to extract the visual pose information only, call getVisualPose()
     * @return (3x4) camera pose matrix with (R | t)
     */
    cv::Mat getPose() const;

    /*!
     * @brief Getter for visually computed pose. Can differ from _cam.pose() if georeference was already computed
     * @return (3x4) camera pose matrix with (R | t)
     */
    cv::Mat getVisualPose() const;

    /*!
     * @brief Getter for geographic pose. Should always be the same as in _cam.pose() or getPose()
     * @return (3x4) camera pose matrix with (R | t)
     */
    cv::Mat getGeographicPose() const;

    /*!
     * @brief Getter for transformation matrix from visual world to geographic coordinate system.
     * @return (4x4) transformation matrix from visual to geographic world.
     */
    cv::Mat getGeoreference() const;

    /*!
     * @brief Getter for the surface assumption
     * @return Surface assumption, e.g. PLANAR or ELEVATION
     */
    SurfaceAssumption  getSurfaceAssumption() const;

    /*!
     * @brief Getter for the observed map, that is a grid in the reference plane
     * @return Grid map of the observed scene
     */
    CvGridMap::Ptr getObservedMap() const;

    /*!
     * @brief Getter for the geotag of the image
     * @return GNSS information struct
     */
    UTMPose getGnssUtm() const;

    /*!
     * @brief Getter for the camera model. Pose of the camera depends on the available information.
     *     If frame pose is accurate, then two options exist:
     *     1) Pose is accurate and georeference was computed -> motion is in the geographic frame
     *     2) Pose is accurate but georeference was not computed -> motion is in visual world frame
     *     If frame pose is not accurate, then return the default pose based on GNSS and heading
     * @return Model for projection, etc of the camera
     */
    camera::Pinhole getCamera() const;

    /*!
     * @brief Getter for the timestamp of the frame
     * @return Timestamp of the frame in nanoseconds
     */
    uint64_t getTimestamp() const;

    /*!
     * @brief Getter for resized image width
     * @return Image width resized depending on the image resize factor set
     */
    uint32_t getResizedImageWidth() const;

    /*!
     * @brief Getter for resized image height
     * @return Image height resized depending on the image resize factor set
     */
    uint32_t getResizedImageHeight() const;

    /*!
     * @brief Getter for resized image size
     * @return Resized image size depending on the image resize factor set
     */
    cv::Size getResizedImageSize() const;

    /*!
     * @brief Getter for the resized, undistorted image
     * @return Resized, undistorted image depending on the image resize factor set
     */
    cv::Mat getResizedImageUndistorted() const;

    /*!
     * @brief Getter for the resized, distorted raw image
     * @return Resized, distorted raw image depending on the image resize factor set
     */
    cv::Mat getResizedImageRaw() const;

    /*!
     * @brief Getter for the resized calibration matrix (pinhole only)
     * @return Resized calibration matrix (pinhole only), that is computed depending on the image resize factor
     */
    cv::Mat getResizedCalibration() const;

    /*!
     * @brief Getter for the resized calibration model
     * @return Resized calibration model, that is computed depending on the image resize factor
     */
    camera::Pinhole getResizedCamera() const;

    /*!
     * @brief Setter for the camera pose computed by either the default pose or more advanced approached, e.g. visual SLAM
     * @param pose 3x4 camera pose matrix
     */
    void setVisualPose(const cv::Mat &pose);

    /*!
     * @brief Setter for the camera pose computed by either the default pose or more advanced approached, e.g. visual SLAM
     * @param pose 3x4 camera pose matrix
     */
    void setGeographicPose(const cv::Mat &pose);

    /*!
     * @brief Setter for transformation from visual world to geographic coordinate frame.
     * @param T_w2g Transformation from visual world (_cam.T_c2w) to geographic
     */
    void setGeoreference(const cv::Mat &T_w2g);

    /*!
     * @brief Setter for surface points, e.g. the sparse or dense cloud observed by the frame. Either computed by the
     *        visual SLAM or an additional densification approach (e.g. stereo reconstruction)
     * @param surface_pts 3D point cloud observed by this frame. Besides camera pose this is the most important parameter
     *        to be computed. It can contain either
     *        1) row(i) = (x, y, z)
     *        2) row(i) = (x, y, z, r, g, b)
     *        3) row(i) = (x, y, z, r, g, b, nx, ny, nz)
     */
    void setSurfacePoints(const cv::Mat &surface_pts);

    /*!
     * @brief Setter for the observed map
     * @param observed_map Observed map is a grid with a defined resolution, that lies in the reference plane (typically
     *        pt = (0,0,0), n = (0,0,1))
     */
    void setObservedMap(const CvGridMap::Ptr &observed_map);

    /*!
     * @brief Call to set this frame as keyframe
     */
    void setKeyframe(bool flag);

    /*!
     * @brief Call to set this frame's pose as accurate. In contrast with a default pose some computations are not
     *        meaningful, e.g. visual dense reconstruction.
     */
    void setPoseAccurate(bool flag);

    /*!
     * @brief Setter for surface assumption. Default is PLANAR, but as soon as surface generation computed a 2.5D
     *        elevation, this function should be called.
     */
    void setSurfaceAssumption(SurfaceAssumption assumption);

    /*!
     * @brief Setter for image resize factor. Many computations can not be done on full sized images. Resizing image and
     *        camera model behind it can be usefull to reduce computational costs. Settings this resize factor is
     *        necessary to call getter for functions with "getResized..." name.
     * @param value Resize factor for image size, e.g. 0.1 means the image is resized to 10% of original edge length,
     *        therefore 1% of the original resolution
     */
    void setImageResizeFactor(const double &value);

    /*!
     * @brief Function to print basic frame informations. Is usefull for debugging at certain points to check if the frame
     *        contains all the informations that it is expected to do.
     * @return string of various frame informations
     */
    std::string print();

    /*!
     * @brief Function to apply a transformation to the current existing informations. Typically this is used to transform
     *        pose and surface points of the frame from the local, visual to a global, georeferenced coordinate system
     * @param T 4x4 homogenous transformation matrix
     */
    void applyGeoreference(const cv::Mat &T);

    /*!
     * @brief Getter to check if this frame is marked as keyframe
     * @return true if yes
     */
    bool isKeyframe() const;

    /*!
     * @brief Getter to check if this frame is already georeferenced
     * @return true if georeferenced
     */
    bool isGeoreferenced() const;

    /*!
     * @brief Getter to check if image resize factor was set
     * @return true if yes
     */
    bool isImageResizeSet() const;

    /*!
     * @brief Getter to check if frame contains observed map data. Reminder: Observed map is the grid map in the reference
     *        plane, which contains informations like elevation, normal, ...
     * @return true if yes
     */
    bool hasObservedMap() const;

    /*!
     * @brief Getter to check if frame has an accurate pose, computed by e.g. visual SLAM
     * @return true if yes
     */
    bool hasAccuratePose() const;

    // DO NOT CALL YOURSELF CURRENTLY
    void updateGeographicPose();

  private:

    /**###########################
     * ########## flags ##########
       ###########################*/

    //! Mutex to protect flags from access
    std::mutex _mutex_flags;

    //! Flag to set frame as keyframe
    bool _is_keyframe;

    //! Flag to set frame georefernced
    bool _is_georeferenced;

    //! Flag if img resized factor was set
    bool _is_img_resizing_set;

    //! Flag if depth of the observed scene is computed
    bool _is_depth_computed;

    //! Flag if frame as accurately computed pose (therefore pose is not default)
    bool _has_accurate_pose;

    //! Flag for surface assumption. Default: PLANAR
    SurfaceAssumption _surface_assumption;


    /**###########################################
     * ########## Image resizing factor ##########
       ###########################################*/

    //! Resize factor can be set by processing pipelines to grab image and calibration scaled to resized image sizes
    double _img_resize_factor;


    /**##########################################
     * ########## Observed scene depth ##########
       ##########################################*/

    //! Minimum scene depth computed from the set surface points. Is computed as soon as "setSurfacePoints" is called
    double _min_scene_depth;

    //! Maximum scene depth computed from the set surface points. Is computed as soon as "setSurfacePoints" is called
    double _max_scene_depth;

    //! Median scene depth computed from the set surface points. Is computed as soon as "setSurfacePoints" is called
    double _med_scene_depth;

    /**###################################################
     * ########## Measured data / apriori infos ##########
       ###################################################*/

    // Measured data constant, no editing, no mutex neccessary
    std::string _camera_id;
    uint32_t _frame_id;
    uint64_t _timestamp;                // timestamp of acquisition in nano seconds
    cv::Mat _img;                       // Image data captured
    UTMPose _utm;                       // GNSS measurement or "GeoTag"

    /**########################################################
     * ########## Data computed after frame creation ##########
       ########################################################*/
    // Note that data computed after frame creation might not be set at certain processing stages. It is also not
    // invariant over all processing steps, which is why it must be protected with mutex

    //! Resized image, resize factor defined through image resize factor, only grabbable if factor was set
    cv::Mat _img_resized;

    //! Reconstructed 3D surface points structured as cv::Mat
    //! Note: Point cloud can be either dense or sparse, and it can contain only positional informations (x,y,z),
    //!       optionally color (x,y,z,r,g,b) or also point normal (x,y,z,r,g,b,nx,ny,nz)
    cv::Mat _surface_pts;

    //! Observed map as grid map in the reference plane [pt = (0,0,0), n = (0,0,1)]. It contains the results of the
    //! reconstruction in the form of an elevation map, normal map, rectified surface color, ...
    CvGridMap::Ptr _observed_map;

    //! Camera model of the frame that performs all the projection work. Currently only pinhole supported
    camera::Pinhole _cam;

    //! Transformation from world to geographic frame
    cv::Mat _T_w2g;

    //! Camera motion in the local world frame
    cv::Mat _M_c2w;

    //! Camera motion in the geographic frame
    cv::Mat _M_c2g;

    //! Mutex for img resized
    std::mutex _mutex_img_resized;

    //! Mutex for surface points
    std::mutex _mutex_surface_pts;

    //! Mutex for observed map
    std::mutex _mutex_observed_map;

    //! Mutex for camera model
    std::mutex _mutex_cam;

    //! Mutex for transformation from world to geographic coordinate frame
    std::mutex _mutex_T_w2g;

    /*!
     * @brief Private function to compute scene depth using the previously set surface points. Can obviously only be
     *        computed if surface points were generated by e.g. visual SLAM or stereo reconstruction. Be careful to
     *        call it. Make sure member "_surface_pts" is really set
     */
    void computeSceneDepth();
};

} // namespace realm


#endif //PROJECT_FRAME_H
