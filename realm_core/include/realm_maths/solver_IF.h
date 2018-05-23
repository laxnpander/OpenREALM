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

#ifndef PROJECT_SOLVER_IF_H
#define PROJECT_SOLVER_IF_H

#include <opencv2/core.hpp>

namespace realm
{

class Solver
{
  public:
    enum class Method
    {
        RANSAC,
        LMED
    };

  public:
    virtual cv::Mat estimate(const cv::Mat &src, const cv::Mat &dst) = 0;
    virtual void setMethod(Method method) = 0;
  private:

};

}

#endif //PROJECT_SOLVER_IF_H
