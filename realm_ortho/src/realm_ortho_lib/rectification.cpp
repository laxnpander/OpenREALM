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

#include <realm_ortho/rectification.h>

using namespace realm;

void ortho::rectify(const Frame::Ptr &frame, CvGridMap &map)
{
  backprojectFromGrid(frame, map);
}

void ortho::backprojectFromGrid(const Frame::Ptr &frame, CvGridMap &map_rect)
{
  // Implementation details:
  // Implementation is chosen as a compromise between readability and performance. Especially the raw array operations
  // could be implemented in opencv. However depending on the resolution of the surface grid and the image the loop
  // iterations can go up to several millions. To keep the computation time as low as possible for this performance sink,
  // the style is as follows
  CvGridMap::Ptr observed_map = frame->getObservedMap();
  assert(observed_map->exists("elevation") && observed_map->get("elevation").type() == CV_32F);
  assert(observed_map->exists("valid") && observed_map->get("valid").type() == CV_8UC1);

  // Prepare projection, use raw arrays for performance
  cv::Mat img = frame->getImageUndistorted();
  cv::Mat cv_P = frame->getCamera()->P();
  double P[3][4] = {cv_P.at<double>(0, 0), cv_P.at<double>(0, 1), cv_P.at<double>(0, 2), cv_P.at<double>(0, 3),
                    cv_P.at<double>(1, 0), cv_P.at<double>(1, 1), cv_P.at<double>(1, 2), cv_P.at<double>(1, 3),
                    cv_P.at<double>(2, 0), cv_P.at<double>(2, 1), cv_P.at<double>(2, 2), cv_P.at<double>(2, 3)};

  // Prepare elevation angle calculation
  cv::Mat t_pose = frame->getCamera()->t();
  double t[3] = {t_pose.at<double>(0), t_pose.at<double>(1), t_pose.at<double>(2)};

  // Get data from container
  double GSD = observed_map->resolution();
  uchar is_elevated = (frame->getSurfaceAssumption() == SurfaceAssumption::PLANAR ? (uchar)0 : (uchar)255);
  cv::Rect2d roi = observed_map->roi();
  cv::Mat elevation = observed_map->get("elevation");
  cv::Mat valid_elevation = observed_map->get("valid");

//  // It seems like at the borders of the elevation map some points don't project into the image, but
//  // others further away do. Consequently a weird "ring" of valid points exists, which is quite ugly.
//  // Further analysis might point out why it behaves like this, maybe due to distortion? ->TODO: Find out why
//  // Hacky solution: Shrink valid area by some pixels to avoid ring
//  cv::erode(valid_elevation, valid_elevation, cv::Mat(), cv::Point(-1, -1), 5, 1, 1);

  // Prepare resulting color layer data
  cv::Mat color_data = cv::Mat::zeros(observed_map->size(), CV_8UC4);
  cv::Mat elevation_angle = cv::Mat::zeros(observed_map->size(), CV_32F);
  cv::Mat elevated = cv::Mat::zeros(observed_map->size(), CV_8UC1);
  cv::Mat num_observations = cv::Mat::zeros(observed_map->size(), CV_16UC1);
  cv::Mat valid_rect = cv::Mat::zeros(observed_map->size(), CV_8UC1);

  // Iterate through
  for (uint32_t r = 0; r < elevation.rows; ++r)
    for (uint32_t c = 0; c < elevation.cols; ++c)
    {
      auto elevation_val = static_cast<double>(elevation.at<float>(r, c));

      if (valid_elevation.at<uchar>(r, c) == 0)
      {
        elevation.at<float>(r, c) = consts::getNoValue<float>();
        continue;
      }
      double pt[3]{roi.x+(double)c*GSD, roi.y-(double)r*GSD, elevation_val};
      double z = P[2][0]*pt[0]+P[2][1]*pt[1]+P[2][2]*pt[2]+P[2][3]*1.0;
      double x = (P[0][0]*pt[0]+P[0][1]*pt[1]+P[0][2]*pt[2]+P[0][3]*1.0)/z;
      double y = (P[1][0]*pt[0]+P[1][1]*pt[1]+P[1][2]*pt[2]+P[1][3]*1.0)/z;
      if (x > 0.0 && x < img.cols && y > 0.0 && y < img.rows)
      {
        color_data.at<cv::Vec4b>(r, c) = img.at<cv::Vec4b>((int)y, (int)x);
        elevation_angle.at<float>(r, c) = static_cast<float>(ortho::internal::computeElevationAngle(t, pt));
        elevated.at<uchar>(r, c) = is_elevated;
        num_observations.at<uint16_t>(r, c) = 1;
        valid_rect.at<uchar>(r, c) = 255;
      }
      else
      {
        elevation.at<float>(r, c) = consts::getNoValue<float>();
      }
    }

  if (map_rect.empty())
    map_rect.setGeometry(observed_map->roi(), observed_map->resolution());
  map_rect.add("color_rgb", color_data);
  map_rect.add("elevation_angle", elevation_angle);
  map_rect.add("elevated", elevated);
  map_rect.add("num_observations", num_observations);
  map_rect.add("valid", valid_rect);
}

double ortho::internal::computeElevationAngle(double *t, double *p)
{
  double v[3]{t[0]-p[0], t[1]-p[1], t[2]-p[2]};
  double v_length = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  return acos(sqrt(v[0]*v[0]+v[1]*v[1])/v_length)*180/3.1415;
}