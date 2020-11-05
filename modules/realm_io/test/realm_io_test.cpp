/**
* This file is part of OpenREALM.
*
* Copyright (C) 2020 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
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

#include <realm_io/realm_import.h>
#include <realm_io/realm_export.h>
#include <realm_io/utilities.h>

// gtest
#include <gtest/gtest.h>

using namespace realm;

TEST(RealmIO, CvGridMapBinary)
{
  // For this test we dump a CvGridMap to a binary file and consequently load it to check whether the content we
  // put in is correct.
  std::string path_tmp = io::getTempDirectoryPath();

  auto map = std::make_shared<CvGridMap>(cv::Rect2d(50.0, 100.0, 200.0, 250.0), 1.0);
  map->add("data_a", cv::Mat(map->size(), CV_8UC4, cv::Scalar(125, 125, 125, 255)), cv::INTER_AREA);
  map->add("data_b", cv::Mat(map->size(), CV_16UC1, 125), cv::INTER_LINEAR);
  map->add("data_c", cv::Mat(map->size(), CV_32FC1, 125.0), cv::INTER_CUBIC);
  map->add("data_d", cv::Mat(map->size(), CV_64FC1, 125.0), cv::INTER_LINEAR);
  io::saveCvGridMap(*map, path_tmp + "/" + "tmp_map.grid.bin");

  auto map_copy = io::loadCvGridMap(path_tmp + "/" + "tmp_map.grid.bin");

  cv::Rect2d roi = map_copy->roi();
  EXPECT_NEAR(roi.x, 50.0, 10e-3);
  EXPECT_NEAR(roi.y, 100.0, 10e-3);
  EXPECT_NEAR(roi.width, 200.0, 10e-3);
  EXPECT_NEAR(roi.height, 250.0, 10e-3);
  EXPECT_NEAR(map_copy->resolution(), 1.0, 10e-3);

  CvGridMap::Layer layer_a = map_copy->getLayer("data_a");
  EXPECT_EQ(layer_a.interpolation, cv::INTER_AREA);
  EXPECT_EQ(layer_a.data.at<cv::Vec4b>(50, 50), cv::Vec4b(125, 125, 125, 255));

  CvGridMap::Layer layer_b = map_copy->getLayer("data_b");
  EXPECT_EQ(layer_b.interpolation, cv::INTER_LINEAR);
  EXPECT_EQ(layer_b.data.at<uint16_t>(50, 50), 125);

  CvGridMap::Layer layer_c = map_copy->getLayer("data_c");
  EXPECT_EQ(layer_c.interpolation, cv::INTER_CUBIC);
  EXPECT_NEAR(layer_c.data.at<float>(50, 50), 125, 10e-3);

  CvGridMap::Layer layer_d = map_copy->getLayer("data_d");
  EXPECT_EQ(layer_d.interpolation, cv::INTER_LINEAR);
  EXPECT_NEAR(layer_d.data.at<double>(50, 50), 125, 10e-3);
}

TEST(RealmIO, CvGridMapBinaryLegacy)
{
  // This test will check, whether the initial binary specification for CvGridMap have changed or not. We do this by
  // loading a file that was created in 11/2020. If something was modified since then, the test will show that.
  auto map = io::loadCvGridMap("test_map.grid.bin");

  cv::Rect2d roi = map->roi();
  EXPECT_NEAR(roi.x, 50.0, 10e-3);
  EXPECT_NEAR(roi.y, 100.0, 10e-3);
  EXPECT_NEAR(roi.width, 200.0, 10e-3);
  EXPECT_NEAR(roi.height, 250.0, 10e-3);
  EXPECT_NEAR(map->resolution(), 1.0, 10e-3);

  CvGridMap::Layer layer_a = map->getLayer("data_a");
  EXPECT_EQ(layer_a.interpolation, cv::INTER_AREA);
  EXPECT_EQ(layer_a.data.at<cv::Vec4b>(50, 50), cv::Vec4b(125, 125, 125, 255));

  CvGridMap::Layer layer_b = map->getLayer("data_b");
  EXPECT_EQ(layer_b.interpolation, cv::INTER_LINEAR);
  EXPECT_EQ(layer_b.data.at<uint16_t>(50, 50), 125);

  CvGridMap::Layer layer_c = map->getLayer("data_c");
  EXPECT_EQ(layer_c.interpolation, cv::INTER_CUBIC);
  EXPECT_NEAR(layer_c.data.at<float>(50, 50), 125, 10e-3);

  CvGridMap::Layer layer_d = map->getLayer("data_d");
  EXPECT_EQ(layer_d.interpolation, cv::INTER_LINEAR);
  EXPECT_NEAR(layer_d.data.at<double>(50, 50), 125, 10e-3);
}

TEST(RealmIO, CameraYaml)
{
  double fps;
  camera::Pinhole cam = io::loadCameraFromYaml("calib.yaml", &fps);

  EXPECT_NEAR(fps, 10.0, 10e-3);
  EXPECT_EQ(cam.width(), 1200);
  EXPECT_NEAR(cam.fx(), 1200.0, 10e-3);
  EXPECT_NEAR(cam.k2(), 0.2, 10e-3);
}