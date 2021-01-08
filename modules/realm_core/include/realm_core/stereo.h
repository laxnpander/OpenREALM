

#ifndef PROJECT_STEREO_H
#define PROJECT_STEREO_H

#include <opencv2/core.hpp>

#include <realm_core/frame.h>
#include <realm_core/camera.h>
#include <realm_core/depthmap.h>

namespace realm
{
namespace stereo
{

void computeRectification(const Frame::Ptr &frame_left,
                          const Frame::Ptr &frame_right,
                          cv::Mat &R1,
                          cv::Mat &P1,
                          cv::Mat &R2,
                          cv::Mat &P2,
                          cv::Mat &Q);

void remap(const Frame::Ptr &frame,
           const cv::Mat &R,
           const cv::Mat &P,
           cv::Mat &img_remapped);

/*!
 * @brief Function for computation of world points from depth map. Depth must be normalised to min/max depth.
 * @param cam Camera model, e.g. pinhole for projection of points. Must contain R, t and K
 * @param depthmap Depth map computed with a stereo reconstruction framework of choice, normalised to min/max depth
 * @return 3-channel double precision matrix (CV_64FC3) with world point coordinates at each element
 */
cv::Mat reprojectDepthMap(const camera::Pinhole::ConstPtr &cam, const cv::Mat &depthmap);

/*!
 * @brief Function for computation of depth and depth map from pointcloud and camera model
 * @param cam Camera model, e.g. pinhole for projection of points. Must contain R, t and K
 * @param points Point cloud structured es mat rowise x, y, z coordinates
 * @return depth map
 */
cv::Mat computeDepthMapFromPointCloud(const camera::Pinhole::ConstPtr &cam, const cv::Mat &points);

/*!
 * @brief Function for computation of normals from an input depth map
 * @param depth Input depth map
 * @return Normal map with type CV_32FC3
 */
cv::Mat computeNormalsFromDepthMap(const cv::Mat& depth);

/*!
 * @brief Computes the baseline between two camera poses using simple euclidean distance
 * @param p1 First pose as transformation matrix 3x4 with (R | t)
 * @param p2 Second pose as transformation matrix 3x4 with (R | t)
 * @return Euclidean distance between first and second pose, aka baseline
 */
double computeBaselineFromPose(const cv::Mat &p1, const cv::Mat &p2);


} // stereo
} // namespace realm

#endif //PROJECT_STEREO_H
