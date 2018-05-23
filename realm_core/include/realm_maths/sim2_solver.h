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

#ifndef PROJECT_SIM2_SOLVER_H
#define PROJECT_SIM2_SOLVER_H

#include <iostream>

#include <opencv2/calib3d.hpp>

#include <realm_maths/solver_IF.h>

namespace realm
{

// @brief: Simple Sim2-Solver interface mostly for opencv functions
//         2D-Similarity transformation is defined by a 3x3 (sR | t) matrix
//
//                  ( s*r11, s*r12, tx)
//         T_sim2 = (-s*r12, s*r11, ty)
//                  ( 0      0      1)
//
//         Therefore 4 DOF: r11, r12, s, tx, ty
//
class Sim2Solver : public Solver
{
  public:
    Sim2Solver();
    cv::Mat estimate(const cv::Mat &src, const cv::Mat &dst) override;
    void setMethod(Solver::Method method) override;

  private:
    Solver::Method _method;
};

}

#endif //PROJECT_SIM2_SOLVER_H
