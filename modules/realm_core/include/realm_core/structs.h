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

#ifndef PROJECT_STRUCTS_H
#define PROJECT_STRUCTS_H

#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

#include <opencv2/core/core.hpp>

#include <realm_core/utm32.h>
#include <realm_core/camera.h>

namespace realm
{

/*!
 * @brief Basic struct for definition of a plane. Might be replaced with a simpler definition in the future (raw arrays)
 */
struct Plane
{
    using Ptr = std::shared_ptr<Plane>;
    using ConstPtr = std::shared_ptr<const Plane>;
    cv::Mat pt;
    cv::Mat n;
};

/*!
 * @brief Basic struct for parameters additionally providing a help description
 * @tparam T Type of the parameter value
 */
template <typename T>
struct Parameter_t
{
    T value;
    std::string help;
};

/*!
 * @brief Basic struct for a face of a mesh
 */
struct Face
{
  cv::Point3d vertices[3];
  cv::Vec4b color[3];
};

}

#endif //PROJECT_STRUCTS_H
