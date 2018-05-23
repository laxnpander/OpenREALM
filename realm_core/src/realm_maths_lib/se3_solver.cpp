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

#include <realm_maths/se3_solver.h>

using namespace realm;

// !!!! WARNING: NOT FULLY IMPLEMENTED YET !!!!
// TODO: implement it!
cv::Mat Se3Solver::estimate(const cv::Mat &src, const cv::Mat &dst)
{
  assert(src.rows == dst.rows);
  assert(src.cols == dst.cols);

  // First define basic eigen variables
  size_t nrof_points = src.rows;
  Eigen::Matrix< Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > src_points(3, nrof_points);
  Eigen::Matrix< Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > dst_points(3, nrof_points);

  for (size_t i = 0; i < nrof_points; ++i)
  {
    src_points.col(i) << src.at<double>(i, 0), src.at<double>(i, 1), src.at<double>(i, 2);
    dst_points.col(i) << dst.at<double>(i, 0), dst.at<double>(i, 1), dst.at<double>(i, 2);
  }

  // Calculate centroid of both pointclouds
  Eigen::Vector3d src_centroid(src_points.row(0).mean(), src_points.row(1).mean(), src_points.row(2).mean());
  Eigen::Vector3d dst_centroid(dst_points.row(0).mean(), dst_points.row(1).mean(), dst_points.row(2).mean());

  Eigen::Matrix3d H(3, 3);
  for (size_t i = 0; i < nrof_points; ++i)
  {
    H += src_points.col(i) * dst_points.col(i).transpose();
  }

  // And correct pointcloud by centroids
  src_points.row(0).array() -= src_centroid(0);
  src_points.row(1).array() -= src_centroid(1);
  src_points.row(2).array() -= src_centroid(2);

  dst_points.row(0).array() -= dst_centroid(0);
  dst_points.row(1).array() -= dst_centroid(1);
  dst_points.row(2).array() -= dst_centroid(2);

  auto svd = H.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d result = (svd.matrixV() * svd.matrixU().transpose());

  return cv::Mat();
}

void Se3Solver::setMethod(Solver::Method method)
{
  // TBD
}