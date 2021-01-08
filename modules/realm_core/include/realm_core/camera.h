

#ifndef PROJECT_CAMERA_H
#define PROJECT_CAMERA_H

#include <memory>
#include <iostream>

#include <opencv2/core/mat.hpp>

namespace realm
{
namespace camera
{

/*!
 * @brief Basic pinhole model class for Open REALM implementation. It is lightweight and makes heavey use of opencv's
 *        cv::Mat. In the future it might be useful to create an abstract class and derive pinhole from that.
 */
class Pinhole
{
  public:
    using Ptr = std::shared_ptr<Pinhole>;
    using ConstPtr = std::shared_ptr<const Pinhole>;
  public:

  /*!
   * @brief Constructor for raw elements of calibration and image size.
   * @param fx Focal length in x
   * @param fy Focal length in y
   * @param cx Principal point in x
   * @param cy Principal point in y
   * @param img_width Width of the image
   * @param img_height Height of the image
   */
    Pinhole(double fx, double fy, double cx, double cy, uint32_t img_width, uint32_t img_height);

    /*!
     * @brief Constructor with opencv like cv::Mat initialization
     * @param K Calibration matrix K with
     *          (fx  0 cx)
     *          ( 0 fy cy)
     *          ( 0  0  1)
     * @param dist_coeffs Distortion coefficients, currently only (k1, k2, p1, p2, 1.0)
     * @param img_width Width of the image
     * @param img_height Height of the image
     */
    Pinhole(const cv::Mat &K, const cv::Mat &dist_coeffs, uint32_t img_width, uint32_t img_height);

    /*!
     * @brief Constructor with opencv like cv::Mat initialization
     * @param K Calibration matrix K with
     *          (fx  0 cx)
     *          ( 0 fy cy)
     *          ( 0  0  1)
     * @param img_width Width of the image
     * @param img_height Height of the image
     */
    Pinhole(const cv::Mat &K, uint32_t img_width, uint32_t img_height);

    /*!
     * @brief Copy constructor
     * @param that Other pinhole camera object
     */
    Pinhole(const Pinhole &that);

    /*!
     * @brief Overload of the assignment operator, deep copying
     * @param that Other pinhole camera object
     * @return Reference to this element
     */
    Pinhole &operator=(const Pinhole &that);

    /*!
     * @brief Getter to check if a lens distortion was set in the constructor
     * @return true if yes
     */
    bool hasDistortion() const;

    /*!
     * @brief Getter for the image width
     * @return Image width
     */
    uint32_t width() const;

    /*!
     * @brief Getter for image height
     * @return Image height
     */
    uint32_t height() const;

    /*!
     * @brief Getter for focal length x
     * @return Camera focal length x
     */
    double fx() const;

    /*!
     * @brief Getter for focal length y
     * @return Camera focal length y
     */
    double fy() const;

    /*!
     * @brief Getter for principal point x-coordinate
     * @return Principal point x-coordinate
     */
    double cx() const;

    /*!
     * @brief Getter for principal point y-coordinate
     * @return Principal point y-coordinate
     */
    double cy() const;

    /*!
     * @brief Getter for lens distortion coefficient k1
     * @return Lens distortion coefficient k1
     */
    double k1() const;

    /*!
     * @brief Getter for lens distortion coefficient k2
     * @return Lens distortion coefficient k2
     */
    double k2() const;

    /*!
     * @brief Getter for lens distortion coefficient p1
     * @return Lens distortion coefficient p1
     */
    double p1() const;

    /*!
     * @brief Getter for lens distortion coefficient p2
     * @return Lens distortion coefficient p2
     */
    double p2() const;

    /*!
     * @brief Getter for lens distortion coefficient k3
     * @return Lens distortion coefficient k3
     */
    double k3() const;

    /*!
     * @brief Getter for camera pose.
     *        Note: Here pose is defined as 3x4 matrix and mapping from camera TO world (direction is important)
     *              See also in contrast: T_c2w is a 4x4 transformation matrix.
     * @return 3x4 (R|t) matrix
     */
    cv::Mat pose() const;

    /*!
     * @brief Getter for calibration matrix K with
     *          (fx  0 cx)
     *          ( 0 fy cy)
     *          ( 0  0  1)
     * @return 3x3 intrinsic camera calibration
     */
    cv::Mat K() const;

    /*!
     * @brief Getter for lens distortion coefficients
     * @return (1x5) matrix with (k1, k2, p1, p2, k3)
     */
    cv::Mat distCoeffs() const;

    /*!
     * @brief Getter for rotation matrix
     * @return (3x3) orthogonal rotation matrix
     */
    cv::Mat R() const;

    /*!
     * @brief Projection matrix to map points from 3D space to 2D image plane
     *              P * X = x
*             with
     *             P = K * (R|t),       // 3x4 proj matrix
     *             X = (x, y, z, 1.0),  // 4x1 homogenous world coordinate
     *             x = (u, v, 1.0)      // 3x1 homogenous image coordinate
     * @return (3x4) projection matrix
     */
    cv::Mat P() const;

    /*!
     * @brief Getter for extrinsic translation vector
     * @return (3x1) vector for euklidean translation of camera in world frame
     */
    cv::Mat t() const;

    /*!
     * @brief Getter for (4x4) homogenous, extrinsic camera parameters (R|t) defined as transformation from camera TO
     *        the world frame.
     * @return (4x4) extrinsic camera parameters (R|t) from camera to world coordinate frame
     */
    cv::Mat Tc2w() const;

    /*!
     * @brief Getter for (4x4) homogenous, extrinsic camera parameters (R|t) defined as transformation from world TO
     *        the camera frame.
     * @return (4x4) extrinsic camera parameters (R|t) from world to camera coordinate frame (Inverse of (R|t))
     */
    cv::Mat Tw2c() const;

    /*!
     * @brief Setter for lens distortion. Directly initializes the rectification maps
     * @param k1 Lens distortion parameter k1
     * @param k2 Lens distortion parameter k2
     * @param p1 Lens distortion parameter p1
     * @param p2 Lens distortion parameter p2
     * @param k3 Lens distortion parameter k3
     */
    void setDistortionMap(const double &k1, const double &k2, const double &p1, const double &p2, const double &k3);

    /*!
     * @brief Setter for lens distortion. Directly initializes the rectification maps
     * @param dist_coeffs Lens distortion coefficients (k1, k2, p1, p2, k3)
     */
    void setDistortionMap(const cv::Mat &dist_coeffs);

    /*!
     * @brief Setter for extrinsic camera parameters. It is defined as pose from camera TO world (direction is important)
     * @param pose (3x4) camera pose
     */
    void setPose(const cv::Mat &pose);

    /*!
     * @brief Function for resizing the current pinhole model to other image sizes. Extensivly used in the different
     *        stages of Open REALM to always adjust the image size to the current problem. That way pose estimation for
     *        example can be computed with another image size than the ortho rectification.
     * @param factor Factor by which the pinhole model should be scaled
     * @return New pinhole model resized by param factor
     */
    Pinhole resize(double factor) const;

    /*!
     * @brief Function for resizing the current pinhole model to other image sizes.
     * @param width Width of the desired image
     * @param height Height of the desired image
     * @return New pinhole model resized to a given set of width and height
     */
    Pinhole resize(uint32_t width, uint32_t height);

    /*!
     * @brief Function to undistort an image with this camera model
     * @param img Image to be undistorted
     * @return Undistorted image
     */
    cv::Mat undistort(const cv::Mat &img, int interpolation) const;

    /*!
     * @brief Function for computation of the distorted image boundaries, therefore
     *              (0, 0)
     *              (0, height)
     *              (width, 0)
     *              (width, height)
     * @return (4x2) matrix with row()=(x, y) containing distorted image boundaries
     */
    cv::Mat computeImageBounds2Ddistorted() const;

    /*!
     * @brief Function for computation of the Undistorted image boundaries, therefore first the image boundaries get
     *        computed, afterwards they are undistorted using the reficitation map.
     * @return (4x2) matrix with row()=(x, y) containing undistorted image boundaries
     */
    cv::Mat computeImageBounds2D() const;

    /*!
     * @brief Computes the intersection of the projected image boundaries with a defined plane in the world frame
     *        and returns four points
     * @param pt One point of the plane
     * @param n A normal vector of the plane
     * @return (4x3) matrix with row()=(x, y, z) containing image boundaries intersected with a plane in the world frame
     */
    cv::Mat projectImageBoundsToPlane(const cv::Mat &pt, const cv::Mat &n) const;

    /*!
     * @brief Computes the intersection of the projected image boundaries with a defined plane in the world frame and
     *        returns a region of interest for the projection
     * @param pt One point of the plane
     * @param n A normal vector of the plane
     * @return Region of interest for the projected camera boundaries
     */
    cv::Rect2d projectImageBoundsToPlaneRoi(const cv::Mat &pt, const cv::Mat &n) const;

    /*!
     * @brief Unprojects a pixel in homogenous coordinates with (x,y,1) to (x,y,z,1) in the world coordinate frame
     * @param x x-coordinate in the image
     * @param y y-coordinate in the image
     * @param depth Depth to which the pixel should be unprojected
     * @return (4x1) homogenous world point
     */
    cv::Mat projectPointToWorld(double x, double y, double depth) const;

  protected:
    // With distortion map
    bool m_do_undistort;

    // Interior parameters
    cv::Mat m_K; // K

    // Distortion parameters
    cv::Mat m_dist_coeffs;
    cv::Mat m_undistortion_map1;
    cv::Mat m_undistortion_map2;

    // Exterior parameters
    cv::Mat m_t; // t
    cv::Mat m_R; // R

    // Image parameters
    uint32_t m_width;
    uint32_t m_height;
};

} // namespace camera

} // namespace realm

#endif