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

#include <iostream>
#include <realm_core/depthmap.h>

#include "test_helper.h"

// gtest
#include <gtest/gtest.h>

using namespace realm;

TEST(Depthmap, Container)
{
  cv::Mat data = cv::Mat(1000, 1200, CV_32F, 33.333);
  camera::Pinhole cam = createDummyPinhole();

  Depthmap depthmap(data, cam);

  EXPECT_NEAR(depthmap.data().at<float>(250, 500), 33.333, 10e-2);
  EXPECT_EQ(depthmap.getCamera()->width(), 1200);
  EXPECT_EQ(depthmap.getCamera()->height(), 1000);
}

TEST(Depthmap, DepthParameters)
{
  cv::Mat data = cv::Mat(1000, 1200, CV_32F, 33.333);
  data.at<float>(50, 50) = 66.666;
  data.at<float>(100, 100) = 11.111;

  camera::Pinhole cam = createDummyPinhole();

  Depthmap depthmap(data, cam);

  EXPECT_NEAR(depthmap.getMedianDepth(), 33.333, 10e-2);
  EXPECT_NEAR(depthmap.getMinDepth(), 11.111, 10e-2);
  EXPECT_NEAR(depthmap.getMaxDepth(), 66.666, 10e-2);
}