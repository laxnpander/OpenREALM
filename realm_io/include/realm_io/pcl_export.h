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

#ifndef PROJECT_PC_EXPORT_H
#define PROJECT_PC_EXPORT_H

#include <iostream>
#include <unordered_map>

#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <opencv2/core.hpp>

#include <realm_core/cv_grid_map.h>


namespace realm
{
namespace io
{

void saveElevationPointsToPLY(const CvGridMap &map,
                              const std::string &ele_layer_name,
                              const std::string &normals_layer_name,
                              const std::string &color_layer_name,
                              const std::string &mask_layer_name,
                              const std::string &directory,
                              const std::string &name);

void saveElevationPoints(const CvGridMap &map,
                         const std::string &ele_layer_name,
                         const std::string &normals_layer_name,
                         const std::string &color_layer_name,
                         const std::string &mask_layer_name,
                         const std::string &filename,
                         const std::string &suffix);

void saveElevationPointsRGB(const CvGridMap &map,
                            const std::string &ele_layer_name,
                            const std::string &color_layer_name,
                            const std::string &mask_layer_name,
                            const std::string &filename,
                            const std::string &suffix);

void saveElevationPointsRGBNormal(const CvGridMap &map,
                                  const std::string &ele_layer_name,
                                  const std::string &normals_layer_name,
                                  const std::string &color_layer_name,
                                  const std::string &mask_layer_name,
                                  const std::string &filename,
                                  const std::string &suffix);

void saveElevationMeshToPLY(const CvGridMap &map,
                            const std::vector<cv::Point2i> &vertices,
                            const std::string &ele_layer_name,
                            const std::string &normal_layer_name,
                            const std::string &color_layer_name,
                            const std::string &mask_layer_name,
                            const std::string &directory,
                            const std::string &name);

void saveElevationMeshToPLY(const CvGridMap &map,
                            const std::vector<cv::Point2i> &vertices,
                            const std::string &ele_layer_name,
                            const std::string &normal_layer_name,
                            const std::string &color_layer_name,
                            const std::string &mask_layer_name,
                            const std::string &filename);

} // namespace io
} // namespace realm



#endif //PROJECT_PC_EXPORT_H
