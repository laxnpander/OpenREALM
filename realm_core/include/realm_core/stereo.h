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

#ifndef PROJECT_STEREO_H
#define PROJECT_STEREO_H

#include <opencv2/core.hpp>

#include <realm_core/frame.h>
#include <realm_core/camera.h>

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
 * @brief Function for computation of world points from depth map. Disparity must be normalised to min/max depth.
 * @param cam Camera model, e.g. pinhole for projection of points. Must contain R, t and K
 * @param depthmap Disparity map computed with a stereo reconstruction framework of choice, normalised to min/max depth
 * @return 3-channel double precision matrix (CV_64FC3) with world point coordinates at each element
 */
cv::Mat reprojectDepthMap(const camera::Pinhole::Ptr &cam,
                          const cv::Mat &depthmap);

/*!
 * @brief Function for computation of depth and depth map from pointcloud and camera model
 * @param cam Camera model, e.g. pinhole for projection of points. Must contain R, t and K
 * @param points Point cloud structured es mat rowise x, y, z coordinates
 * @param depth Output depth map as cv::Mat
 */
void computeDepthMapFromPointCloud(const camera::Pinhole::Ptr &cam,
                                   const cv::Mat &points,
                                   cv::Mat &depth);

/*!
 * @brief Function for computation of normals from an input depth map
 * @param depth Input depth map
 * @return Normal map
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
