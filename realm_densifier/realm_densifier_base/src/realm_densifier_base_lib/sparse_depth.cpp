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

#include <realm_densifier_base/sparse_depth.h>

using namespace realm;

void densifier::computeDepthMapFromSparseCloud(const cv::Mat &sparse_cloud,
                                               const camera::Pinhole::Ptr &cam,
                                               cv::OutputArray out_depth,
                                               cv::OutputArray out_thumbnail)
{
  // Neighbourhood radius is set to 1% of the input image but at least to 3
  double radius = 0.01* static_cast<double>(cam->width());
  if (radius < 3.0)
    radius = 3.0;

  cv::Mat depth_sparse;
  stereo::computeDepthMapFromPointCloud(cam, sparse_cloud, depth_sparse);

  // Optional output thumbnail:
  if (out_thumbnail.needed())
    out_thumbnail.assign(depth_sparse.clone());

  cv::Mat mask_inpaint = (depth_sparse < 0);
  realm::inpaint(depth_sparse, mask_inpaint, depth_sparse, radius, INPAINT_NS);

  // Note: Inpainting with navier stokes produces strange visual artifacts that may result from different properties
  //       of the equation. These artifacts are characterized by local "salt and pepper noise"'ish regions. Apply
  //       median filter to reduce this effect.
  cv::medianBlur(depth_sparse, depth_sparse, 5);

  // Output sparse disparity
  out_depth.assign(depth_sparse);
}

void densifier::computeSparseMask(const cv::Mat &sparse_cloud,
                                  const camera::Pinhole::Ptr &cam,
                                  cv::OutputArray out_mask)
{
  cv::Mat depth_sparse;
  stereo::computeDepthMapFromPointCloud(cam, sparse_cloud, depth_sparse);

  cv::Mat mask_valid = densifier::internal::computeBoundingPolygon(depth_sparse);

  out_mask.assign(mask_valid);
}

cv::Mat densifier::internal::computeBoundingPolygon(const cv::Mat &map)
{
  // Idea: Create a bounding box of all valid sparse points to avoid interpolations way beyond reasonable data

  // Therefore first compute the minimum bounding rectangle from all points with disparity
  cv::Mat mask_nonzero = (map > 0);
  std::vector<cv::Point2i> points;
  for (int r = 0; r < mask_nonzero.rows; ++r)
    for (int c = 0; c < mask_nonzero.cols; ++c)
      if (mask_nonzero.at<uchar>(r, c) == 255)
        points.emplace_back(cv::Point2i(c, r));
  cv::RotatedRect roi = cv::minAreaRect(points);

  // Then use this rotated rectangle to create a mask with convex filling of polygon
  cv::Mat mask = cv::Mat::zeros(mask_nonzero.rows, mask_nonzero.cols, CV_8UC1);
  cv::Point2f vertices2f[4];
  cv::Point vertices[4];
  roi.points(vertices2f);
  for(int i = 0; i < 4; ++i){
    vertices[i] = vertices2f[i];
  }
  cv::fillConvexPoly(mask, vertices, 4, cv::Scalar(255, 0, 0));
  return mask;
}