

#ifndef PROJECT_SPARSE_DISPARITY_H
#define PROJECT_SPARSE_DISPARITY_H

#include <opencv2/core.hpp>

#include <realm_core/camera.h>
#include <realm_core/stereo.h>
#include <realm_core/inpaint.h>

namespace realm
{
namespace densifier
{

/*!
 * @brief Function to compute dense depth map from a sparse point cloud. Uses inpainting to interpolate the regions
 *        in between the back projected sparse points.
 * @param sparse_cloud Input sparse cloud. Should not be too sparse, otherwise interpolation will be very bad
 * @param cam Pinhole camera model for back projection
 * @param out_depth Dense output depth map
 * @param out_thumbnail Thumbnail is the sparse cloud back projected into the image plane without inpainting
 */
void computeDepthMapFromSparseCloud(const cv::Mat &sparse_cloud,
                                    const camera::Pinhole::Ptr &cam,
                                    cv::OutputArray out_depth,
                                    cv::OutputArray out_thumbnail = cv::Mat());

/*!
 * @brief Function to compute a mask from a sparse cloud. Points are reprojected into the image plane and afterwards
 *        a bounding polygon around all valid pixels is created.
 * @param sparse_cloud Sparse cloud from which the mask should be computed
 * @param cam Pinhole camera for reprojection
 * @param out_mask Output mask
 */
void computeSparseMask(const cv::Mat &sparse_cloud,
                       const camera::Pinhole::Ptr &cam,
                       cv::OutputArray out_mask);

namespace internal
{

/*!
 * @brief Function to create a bounding polygon around all valid sparse points.
 * @param map Sparse depth map
 * @return Bounded polygon as mask
 */
cv::Mat computeBoundingPolygon(const cv::Mat &map);

}

} // namespace densifier
} // namespace realm

#endif //PROJECT_SPARSE_DISPARITY_H
