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

#ifndef PROJECT_REALM_EXPORT_H
#define PROJECT_REALM_EXPORT_H

#include <fstream>

#include <eigen3/Eigen/Eigen>

#include <realm_core/cv_grid_map.h>
#include <realm_core/camera.h>
#include <realm_io/utilities.h>

namespace realm
{
namespace io
{

void saveTimestamp(uint64_t timestamp,
                   uint32_t frame_id,
                   const std::string &directory,
                   const std::string &name);

void saveTimestamp(uint64_t timestamp,
                   uint32_t frame_id,
                   const std::string &filename);

void saveTrajectory(uint64_t timestamp,
                    const cv::Mat &pose,
                    const std::string &directory,
                    const std::string &name);

void saveTrajectoryTUM(std::ofstream *file,
                       uint64_t timestamp,
                       double x,
                       double y,
                       double z,
                       double qx,
                       double qy,
                       double qz,
                       double qw);

} // namespace io
} // namespace realm

#endif //PROJECT_REALM_EXPORT_H
