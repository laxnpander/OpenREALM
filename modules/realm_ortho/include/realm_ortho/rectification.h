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

#ifndef PROJECT_RECTIFICATION_H
#define PROJECT_RECTIFICATION_H

#include <memory>
#include <string>
#include <cmath>

#include <realm_core/structs.h>
#include <realm_core/frame.h>
#include <realm_core/cv_grid_map.h>

namespace realm
{
namespace ortho
{

/*!
 * @brief Rectification for elevation assumption. This is a 2.5D surface, that has one elevation value per grid element.
 * @param frame container for aerial measurement data. Here mainly the image and the camera model must be set.
 * @return
 */
CvGridMap::Ptr rectify(const Frame::Ptr &frame);

/*!
 * @brief Projects every element of a grid structure with its world coordinate consisting of (utm east, utm north,
 *        elevation) back int the camera and sets corresponding value in the grid with the color of the image.
 * @param frame container for aerial measurement data. Here mainly the image and the camera model must be set.
 * @param surface container for the surface structure, also known as digital surface model (DSM). Current implementation
 *        needs a layer named "elevation", which is further used for backprojection from grid structure.
 * @param map result is written into this parameter, it contains the layer "color_rgb" and "observation angle"
 */
CvGridMap::Ptr backprojectFromGrid(
    const cv::Mat &img,
    const camera::Pinhole &cam,
    const cv::Mat &surface,
    const cv::Mat &valid_surface,
    const cv::Rect2d &roi,
    double GSD,
    bool is_elevated);

namespace internal
{

inline double computeElevationAngle(double t[3], double p[3]);

} // namespace internal
} // namespace ortho
} // namespace realm

#endif //PROJECT_RECTIFICATION_H
