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

#include "test_helper.h"

using namespace realm;

camera::Pinhole createDummyPinhole()
{
  int width = 1250;
  int height = 623;

  cv::Mat K(3, 3, CV_64F);
  K.at<double>(0, 0) = 1000.0;
  K.at<double>(1, 1) = 1100.0;
  K.at<double>(0, 2) = 600.0;
  K.at<double>(1, 2) = 300.0;

  cv::Mat distortion(5, 1, CV_64F);
  distortion.at<double>(0) = 0.001;
  distortion.at<double>(0) = 0.002;
  distortion.at<double>(0) = 0.003;
  distortion.at<double>(0) = 0.004;
  distortion.at<double>(0) = 0.005;

  return camera::Pinhole(K, distortion, width, height);
}