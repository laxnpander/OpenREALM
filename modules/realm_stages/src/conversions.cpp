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

#include <realm_stages/conversions.h>

namespace realm
{

cv::Mat cvtToPointCloud(const cv::Mat &img3d, const cv::Mat &color, const cv::Mat &normals, const cv::Mat &mask)
{
  assert(!img3d.empty());

  size_t n = (size_t)img3d.cols*img3d.rows;

  cv::Mat points;
  points.reserve(n);

  // Create one point per valid grid element
  for (int r = 0; r < img3d.rows; ++r)
    for (int c = 0; c < img3d.cols; ++c)
    {
      if (mask.at<uchar>(r, c) == 0)
        continue;

      cv::Mat pt = cv::Mat::zeros(1, 9, CV_64F);

      // Position informations
      auto pos = img3d.at<cv::Vec3d>(r, c);
      pt.at<double>(0) = pos[0];
      pt.at<double>(1) = pos[1];
      pt.at<double>(2) = pos[2];

      // Color informations
      if (!color.empty())
      {
        if (color.channels() == 1)
        {
          auto mono = color.at<uchar>(r, c);
          pt.at<double>(3) = static_cast<double>(mono)/255.0;
          pt.at<double>(4) = static_cast<double>(mono)/255.0;
          pt.at<double>(5) = static_cast<double>(mono)/255.0;
        }
        else if(color.channels() == 3)
        {
          auto bgr = color.at<cv::Vec3b>(r, c);
          pt.at<double>(3) = static_cast<double>(bgr[0])/255.0;
          pt.at<double>(4) = static_cast<double>(bgr[1])/255.0;
          pt.at<double>(5) = static_cast<double>(bgr[2])/255.0;
        }
        else if(color.channels() == 4)
        {
          auto bgra = color.at<cv::Vec4b>(r, c);
          pt.at<double>(3) = static_cast<double>(bgra[0])/255.0;
          pt.at<double>(4) = static_cast<double>(bgra[1])/255.0;
          pt.at<double>(5) = static_cast<double>(bgra[2])/255.0;
        }
        else
          throw(std::invalid_argument("Error: Converting to colored pointcloud failed. Image channel mismatch!"));
      }

      // Surface normals
      if (!normals.empty())
      {
        cv::Vec3f normal = normals.at<cv::Vec3f>(r, c);
        pt.at<double>(6) = static_cast<double>(normal[0]);
        pt.at<double>(7) = static_cast<double>(normal[1]);
        pt.at<double>(8) = static_cast<double>(normal[2]);
      }

      points.push_back(pt);
    }
  return points;

}

cv::Mat cvtToPointCloud(const CvGridMap &map,
                        const std::string &layer_elevation,
                        const std::string &layer_color,
                        const std::string &layer_normals,
                        const std::string &layer_mask)
{
  assert(map.exists(layer_elevation));
  assert(!layer_color.empty() ? map.exists(layer_color) : true);
  assert(!layer_normals.empty() ? map.exists(layer_normals) : true);
  assert(!layer_mask.empty() ? map.exists(layer_mask) : true);

  cv::Size2i size = map.size();
  size_t n = (size_t)size.width*size.height;

  cv::Mat points;
  points.reserve(n);

  // OPTIONAL
  cv::Mat color;
  if (map.exists(layer_color))
    color = map[layer_color];

  cv::Mat elevation_normal;
  if (map.exists(layer_normals))
    elevation_normal = map[layer_normals];

  cv::Mat mask;
  if (map.exists(layer_mask))
    mask = map[layer_mask];

  // Create one point per grid element
  cv::Mat img3d(size, CV_64FC3);
  for (int r = 0; r < size.height; ++r)
    for (int c = 0; c < size.width; ++c)
      img3d.at<cv::Point3d>(r, c) = map.atPosition3d(r, c, layer_elevation);

  return cvtToPointCloud(img3d, color, elevation_normal, mask);
}

std::vector<Face> cvtToMesh(const CvGridMap &map,
                            const std::string &layer_elevation,
                            const std::string &layer_color,
                            const std::vector<cv::Point2i> &vertex_ids)
{
  assert(!layer_elevation.empty() && map.exists(layer_elevation));
  assert(!layer_color.empty() ? map.exists(layer_color) : true);

  // OPTIONAL
  cv::Mat color;
  if (map.exists(layer_color))
    color = map[layer_color];

  // Create output vector of faces
  size_t count = 0;
  std::vector<Face> faces(vertex_ids.size()/3);

  for (size_t i = 0; i < vertex_ids.size(); i+=3, ++count)
    for (size_t j = 0; j < 3; ++j)
    {
      faces[count].vertices[j] = map.atPosition3d(vertex_ids[i+j].y, vertex_ids[i+j].x, layer_elevation);
      if (!color.empty())
        faces[count].color[j] = color.at<cv::Vec4b>(vertex_ids[i+j].y, vertex_ids[i+j].x);
      else
        faces[count].color[j] = cv::Vec4b(0, 0, 0, 255);
    }
  return faces;
}

} // namespace realm