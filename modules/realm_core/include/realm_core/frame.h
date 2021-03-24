

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
#include <realm_core/depthmap.h>
#include <realm_core/point_cloud.h>

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
     * @param orientation Orientation of the camera (not the vehicle)
     */
    Frame(const std::string &camera_id,
          const uint32_t &frame_id,
          const uint64_t &timestamp,
          const cv::Mat &img,
          const UTMPose &utm,
          const camera::Pinhole::Ptr &cam,
          const cv::Mat &orientation);

    /*
     * @brief Frame represents a unique container of acquired and computed data and is therefore designed to be non-copyable.
     * Please create it as shared pointer and pass it around your system as such!
     */
    Frame(const Frame &f) = delete;
    void operator=(const Frame &) = delete;

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
     * @brief Getter for the depthmap which is usually computed during the densification stage.
     * @return Depthmap of the observed scene. Not necessarily the same size as the image data.
     */
    Depthmap::Ptr getDepthmap() const;

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
     * @return Sparse reconstructed point cloud set inside the visual SLAM framework.
     */
    PointCloud::Ptr getSparseCloud() const;

    /*!
     * @brief Getter for frame pose. Is always up to date, therefore either the default, visual or georeferenced pose
     *        If you want to extract the visual pose information only, call getVisualPose()
     * @return (3x4) camera pose matrix with (R | t)
     */
    cv::Mat getPose() const;

    /*!
     * @brief Getter for visually computed pose. Can differ from _camera_model.pose() if georeference was already computed
     * @return (3x4) camera pose matrix with (R | t)
     */
    cv::Mat getVisualPose() const;

    /*!
     * @brief Getter for geographic pose. Should always be the same as in _camera_model.pose() or getPose()
     * @return (3x4) camera pose matrix with (R | t)
     */
    cv::Mat getGeographicPose() const;

    /*!
     * @brief Getter for the provided camera orientation. It is assumed to be measured by another source than the visual
     * SLAM.
     * @return 3x3 orientation matrix of the camera
     */
    cv::Mat getOrientation() const;

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
     * @brief Getter for the surface model, which is a grid in the reference plane
     * @return Multi-layered grid map representing the observed surface
     */
    CvGridMap::Ptr getSurfaceModel() const;

    /*!
     * @brief Getter for the rectified orthophoto, which is a grid map in the reference plane
     * @return Single-layered grid map representing the rectified color information of the observed scene. Usually the
     * the resolution is higher than the one of the surface model, so check before you compare the data.
     */
    CvGridMap::Ptr getOrthophoto() const;

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
    camera::Pinhole::ConstPtr getCamera() const;

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
    camera::Pinhole::Ptr getResizedCamera() const;

    /*!
     * @brief Setter for the camera pose computed by either the default pose or more advanced approached, e.g. visual SLAM
     * @param pose 3x4 camera pose matrix
     */
    void setVisualPose(const cv::Mat &pose);

    /*!
     * @brief Setter for surface points, e.g. the sparse cloud observed by the frame. Usually it is computed by the
     *        visual SLAM.
     * @param sparse_cloud 3D point cloud observed by this frame. Besides camera pose this is the most important parameter
     *        to be computed. It can contain either
     *        1) row(i) = (x, y, z)
     *        2) row(i) = (x, y, z, r, g, b)
     *        3) row(i) = (x, y, z, r, g, b, nx, ny, nz)
     */
    void setSparseCloud(const PointCloud::Ptr &sparse_cloud, bool in_visual_coordinates);

    /*!
     * @brief Setter for depthmap. Invalid depth values are set to -1.0.
     * @param depthmap Depthmap of the observed scene.
     */
    void setDepthmap(const Depthmap::Ptr &depthmap);

    /*!
     * @brief Setter for the digital surface model
     * @param surface_model Surface model is a grid with a defined resolution, that lies in the reference plane (typically
     *        pt = (0,0,0), n = (0,0,1))
     */
    void setSurfaceModel(const CvGridMap::Ptr &surface_model);

    /*!
     * @brief Setter for the orthophoto
     * @param orthophoto Single-layered grid map containing a ["color_rgb"] layer with the rectified image data
     */
    void setOrthophoto(const CvGridMap::Ptr &orthophoto);

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
     * @brief Function to apply a transformation to the current existing frame data. Typically this is used to transform
     *        pose and sparse point cloud of the frame from the local, visual to a global, georeferenced coordinate system.
     *        Remember to use only 'updateGeoreference(...)' after that, as it does not transform the sparse cloud.
     * @param T 4x4 homogenous transformation matrix
     */
    void initGeoreference(const cv::Mat &T);

    /*!
     * @brief Updates the transformation from the world to the geographic frame (T_w2g) and updates the georeferenced
     * poses accordingly. It can also be used as a setter for the georeference. If you also want to apply the georeference
     * to the observed sparse cloud, use the function 'initGeoreference(...)'. But make sure the sparse cloud are
     * in the camera coordinate system!
     * @param T 4x4 homogenous transformation matrix from the world to the geographic frame
     * @param do_update_sparse_cloud By setting this flag to false, you can use this function basically as a setter
     * for the georeference. Be cautious when to set it true! If the sparse cloud are already in a geographic frame
     * and no prior georeference was set, calling this update will basically double apply the georeference to the points.
     */
    void updateGeoreference(const cv::Mat &T, bool do_update_sparse_cloud = false);

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
     * @brief Getter to check if scene depth was computed
     * @return true if yes
     */
    bool isDepthComputed() const;

  /*!
   * @brief Getter to check if frame has an accurate pose, computed by e.g. visual SLAM
   * @return true if yes
   */
    bool hasAccuratePose() const;

  private:

    /**###########################
     * ########## flags ##########
       ###########################*/

    //! Mutex to protect flags from access
    mutable std::mutex m_mutex_flags;

    //! Flag to set frame as keyframe
    bool m_is_keyframe;

    //! Flag to set frame georefernced
    bool m_is_georeferenced;

    //! Flag if img resized factor was set
    bool m_is_img_resizing_set;

    //! Flag if depth of the observed scene is computed
    bool m_is_depth_computed;

    //! Flag if frame as accurately computed pose (therefore pose is not default)
    bool m_has_accurate_pose;

    //! Flag for surface assumption. Default: PLANAR
    SurfaceAssumption m_surface_assumption;

    //! Resize factor can be set by processing pipelines to grab image and calibration scaled to resized image sizes
    double m_img_resize_factor;


    /**##########################################
     * ########## Observed scene depth ##########
       ##########################################*/

    //! Minimum scene depth computed from the set surface points. Is computed as soon as at least a sparse cloud was set
    double m_min_depth;

    //! Maximum scene depth computed from the set surface points. Is computed as soon as at least a sparse cloud was set
    double m_max_depth;

    //! Median scene depth computed from the set surface points. Is computed as soon as at least a sparse cloud was set
    double m_med_depth;

    Depthmap::Ptr m_depthmap;

    /**###################################################
     * ########## Measured data / apriori infos ##########
       ###################################################*/

    // Measured data constant, no editing, no mutex neccessary
    std::string m_camera_id;             // (required) Unique camera identifier
    uint32_t m_frame_id;                 // (required) Unique frame identifier
    uint64_t m_timestamp;                // (required) timestamp of acquisition in nano seconds
    cv::Mat m_img;                       // (required) Image data captured
    UTMPose m_utm;                       // (required) GNSS measurement or "GeoTag"
    cv::Mat m_orientation;               // (optional) Orientation of the camera

    /**########################################################
     * ########## Data computed after frame creation ##########
       ########################################################*/
    // Note that data computed after frame creation might not be set at certain processing stages. It is also not
    // invariant over all processing steps, which is why it must be protected with mutex

    //! Resized image, resize factor defined through image resize factor, only grabbable if factor was set
    cv::Mat m_img_resized;

    //! Reconstructed 3D sparse cloud containing data (cv::Mat with row(i) = x, y, z, r, g, b, nx, ny, nz), point ids as
    //! well as a unique context identifier. The Ids refer to the ids that are assigned inside the visual SLAM, so points
    //! are uniquely identified inside a context. The context id is typically required, because after a reset SLAM systems
    //! may also reset the point ids. So to be able to unqiuely identify a point the context id can be used.
    PointCloud::Ptr m_sparse_cloud;

    //! Digital surface model of the observed scene represented by a 2.5D grid map. Layers contained:
    //! Essential:
    //! ["elevation"]: Containing the elevation data per grid cell
    //! Optional:
    //! [...]
    CvGridMap::Ptr m_surface_model;

    //! Orthophoto of the observed scene. Layers contained:
    //! ["color_rgb"]: Containing the rectified RGB data per grid cell
    CvGridMap::Ptr m_orthophoto;

    //! Camera model of the frame that performs all the projection work. Currently only pinhole supported
    camera::Pinhole::Ptr m_camera_model;

    //! 4x4 transformation from world to geographic UTM frame
    cv::Mat m_transformation_w2g;

    //! 3x4 camera motion matrix in the local world frame
    cv::Mat m_motion_c2w;

    //! 3x4 camera motion in the geographic frame
    cv::Mat m_motion_c2g;

    //! Mutex for img resized
    mutable std::mutex m_mutex_img_resized;

    //! Mutex for sparse cloud
    mutable std::mutex m_mutex_sparse_points;

    //! Mutex for the digital surface model
    mutable std::mutex m_mutex_surface_model;

    //! Mutex for orthophoto
    mutable std::mutex m_mutex_orthophoto;

    //! Mutex for camera model
    mutable std::mutex m_mutex_cam;

    //! Mutex for transformation from world to geographic coordinate frame
    mutable std::mutex m_mutex_T_w2g;

    /*!
     * @brief Private function to compute scene depth using the previously set sparse cloud. Can obviously only be
     *        computed if sparse cloud was generated by e.g. visual SLAM or stereo reconstruction. Be careful to
     *        call it. Make sure member "_surface_points" is really set
     *        @param max_nrof_points The maximum number of points on which the scene depth is computed. When set to 0,
     *        all points are used. Note that in case of a dense cloud of 100 000 the computation can be time consuming.
     *        The selected points are sampled across the whole vector to (hopefully) achieve uniformity.
     */
    void computeSceneDepth(int max_nrof_points);

    /*!
     * @brief Setter for transformation from visual world to geographic coordinate frame.
     * @param T_w2g Transformation from visual world (_camera_model.T_c2w) to geographic
     */
    void setGeoreference(const cv::Mat &T_w2g);

    /*!
    * @brief Setter for the camera pose computed by either the default pose or more advanced approached, e.g. visual SLAM
    * @param pose 3x4 camera pose matrix
    */
    void setGeographicPose(const cv::Mat &pose);

    /*!
    * @brief: Uses the provided 4x4 matrix to transform the sparse cloud into a new coordinate system
    * @param T 4x4 transformation matrix
    */
    void applyTransformationToSparseCloud(const cv::Mat &T);

    /*!
     * @brief: Uses the provided 4x4 matrix to transform the visual pose into a new coordinate system
     * @param T 4x4 transformation matrix
     * @return Transformed 3x4 pose matrix
    */
    cv::Mat applyTransformationToVisualPose(const cv::Mat &T);

    /*!
     * @brief Computes the difference between the existing and a newly applied georeference. Because their might be a
     * scale change this is not straight forward. First scale has to be removed to safely invert the old transformation.
     * Then T_old.inv() * T_new is computed to identify the difference between the two transformations.
     * @param T_old Previous georeference as 4x4 transformation matrix
     * @param T_new New georeference to be applied as 4x4 transformation matrix
     * @return Difference of transformation T_old and T_new
     */
    cv::Mat computeGeoreferenceDifference(const cv::Mat &T_old, const cv::Mat &T_new);
};

} // namespace realm


#endif //PROJECT_FRAME_H
