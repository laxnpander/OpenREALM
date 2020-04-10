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
                                    const camera::Pinhole &cam,
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
                       const camera::Pinhole &cam,
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
