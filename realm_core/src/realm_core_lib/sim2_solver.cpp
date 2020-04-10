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

#include <realm_core/sim2_solver.h>

using namespace realm;

Sim2Solver::Sim2Solver()
{
  _method = Solver::Method::RANSAC;
}

cv::Mat Sim2Solver::estimate(const cv::Mat &src, const cv::Mat &dst)
{
  assert(src.type() == CV_64F);
  assert(dst.type() == CV_64F);
  assert(src.rows == dst.rows);
  assert(src.cols == dst.cols);

  std::vector<cv::Point2f> src_pts;
  std::vector<cv::Point2f> dst_pts;

  for (size_t i = 0; i < src.rows; ++i)
  {
    cv::Point2f pt_src;
    pt_src.x = (float)src.at<double>(i, 0);
    pt_src.y = (float)src.at<double>(i, 1);
    src_pts.push_back(pt_src);

    cv::Point2f pt_dst;
    pt_dst.x = (float)dst.at<double>(i, 0);
    pt_dst.y = (float)dst.at<double>(i, 1);
    dst_pts.push_back(pt_dst);
  }

  cv::Mat H_sim2;
  switch (_method)
  {
    case Solver::Method::RANSAC:
      H_sim2 = cv::estimateAffinePartial2D(src_pts, dst_pts, cv::noArray(), cv::RANSAC);
      break;
    case Solver::Method::LMED:
      H_sim2 = cv::estimateAffinePartial2D(src_pts, dst_pts, cv::noArray(), cv::LMEDS);
  }

  cv::Mat hom = (cv::Mat_<double>(1, 3) << 0.0, 0.0, 1.0);
  H_sim2.convertTo(H_sim2, CV_64F);
  H_sim2.push_back(hom);

  return H_sim2;
}

void Sim2Solver::setMethod(Method method)
{
  _method = method;
}