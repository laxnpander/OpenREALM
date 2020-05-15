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

// gtest
#include <gtest/gtest.h>

cv::Mat realm::createDummyPose()
{
  cv::Mat pose = cv::Mat::zeros(3, 4, CV_64F);

  // Rotation: We create a rotation, that is 90° tilted around the z-axis
  pose.at<double>(0, 1) = 1.0;
  pose.at<double>(1, 0) = 1.0;
  pose.at<double>(2, 2) = -1.0;

  // Translation: We create a translation, that is offsetted in y-axis
  pose.at<double>(0, 3) = 500.0;
  pose.at<double>(1, 3) = 600.0;
  pose.at<double>(2, 3) = 1200.0;

  return pose;
}

realm::camera::Pinhole realm::createDummyPinhole()
{
  int width = 1200;
  int height = 1000;

  cv::Mat K(3, 3, CV_64F);
  K.at<double>(0, 0) = 1200.0;
  K.at<double>(1, 1) = 1200.0;
  K.at<double>(0, 2) = 600.0;
  K.at<double>(1, 2) = 500.0;
  K.at<double>(2, 2) = 1.0;

  cv::Mat distortion(5, 1, CV_64F);
  distortion.at<double>(0) = 0.0;
  distortion.at<double>(1) = 0.0;
  distortion.at<double>(2) = 0.0;
  distortion.at<double>(3) = 0.0;
  distortion.at<double>(4) = 0.0;

  return camera::Pinhole(K, distortion, width, height);
}

realm::Frame::Ptr realm::createDummyFrame()
{
  // Set all the data elements on acquisition
  std::string camera_id = "DUMMY_CAM";
  uint32_t frame_id = 123456;
  uint64_t timestamp = 1234567890;
  cv::Mat img = cv::Mat::ones(1000, 1200, CV_8UC3) * 125;
  UTMPose utm(603976, 5791569, 100.0, 45.0, 32, 'U');
  auto cam = std::make_shared<camera::Pinhole>(createDummyPinhole());

  // Create the dummy frame -> We usually use it as shared pointer, so test that here
  auto frame = std::make_shared<Frame>(camera_id, frame_id, timestamp, img, utm, cam);
  frame->setPoseAccurate(true);
  frame->setKeyframe(true);
  frame->setImageResizeFactor(0.5);
  return frame;
}