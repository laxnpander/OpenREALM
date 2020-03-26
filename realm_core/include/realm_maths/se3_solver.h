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

#ifndef PROJECT_SE3_SOLVER_H
#define PROJECT_SE3_SOLVER_H

#include <opencv2/core.hpp>

#include <realm_maths/solver_IF.h>


namespace realm
{

class Se3Solver : public Solver
{
  public:
    cv::Mat estimate(const cv::Mat &src, const cv::Mat &dst) override;
    void setMethod(Solver::Method method) override;

  private:
};

}

#endif //PROJECT_SE3_SOLVER_H
