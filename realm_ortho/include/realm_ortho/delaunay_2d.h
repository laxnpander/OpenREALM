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

#ifndef PROJECT_DELAUNAY_2D_H
#define PROJECT_DELAUNAY_2D_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <realm_types/structs.h>
#include <realm_types/cv_grid_map.h>

namespace realm
{

class Delaunay2D
{
  public:
    using Ptr = std::shared_ptr<Delaunay2D>;
    using ConstPtr = std::shared_ptr<const Delaunay2D>;
  public:
    explicit Delaunay2D();
    std::vector<cv::Point2i> buildMesh(const CvGridMap &grid, const std::string &mask = "");
};

} // namespace realm

#endif //PROJECT_DELAUNAY_2D_H
