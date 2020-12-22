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

#include <realm_io/cv_import.h>
#include <realm_io/cv_export.h>
#include <realm_io/utilities.h>

// gtest
#include <gtest/gtest.h>

using namespace realm;

TEST(CvIO, ImageBinary)
{
  // For this test we dump an opencv matrix to a binary file and consequently load it to check whether the content we
  // put in is still correct.

  std::string path_tmp = io::getTempDirectoryPath();

  cv::Mat img_8uc4(500, 1000, CV_8UC4, cv::Scalar(125,125,125, 255));
  cv::Mat img_16uc1(500, 1000, CV_16UC1, cv::Scalar(125));
  cv::Mat img_32fc1(500, 1000, CV_32FC1, cv::Scalar(125.0));
  cv::Mat img_64fc1(500, 1000, CV_64FC1, cv::Scalar(125.0));

  io::saveImage(img_8uc4,  path_tmp + "/" + "img_8uc4.bin");
  io::saveImage(img_16uc1, path_tmp + "/" + "img_16uc1.bin");
  io::saveImage(img_32fc1, path_tmp + "/" + "img_32fc1.bin");
  io::saveImage(img_64fc1, path_tmp + "/" + "img_64fc1.bin");

  cv::Mat img_8uc4_copy  = io::loadImage(path_tmp + "/" + "img_8uc4.bin");
  cv::Mat img_16uc1_copy = io::loadImage(path_tmp + "/" + "img_16uc1.bin");
  cv::Mat img_32fc1_copy = io::loadImage(path_tmp + "/" + "img_32fc1.bin");
  cv::Mat img_64fc1_copy = io::loadImage(path_tmp + "/" + "img_64fc1.bin");

  EXPECT_EQ(img_8uc4_copy.at<cv::Vec4b>(100, 100), cv::Vec4b(125, 125, 125, 255));
  EXPECT_EQ(img_16uc1_copy.at<uint16_t>(100, 100), 125);
  EXPECT_NEAR(img_32fc1_copy.at<float>(100, 100), 125.0, 10e-3);
  EXPECT_NEAR(img_64fc1_copy.at<double>(100, 100), 125.0, 10e-3);

  io::removeFileOrDirectory(path_tmp + "/" + "img_8uc4.bin");
  io::removeFileOrDirectory(path_tmp + "/" + "img_16uc1.bin");
  io::removeFileOrDirectory(path_tmp + "/" + "img_32fc1.bin");
  io::removeFileOrDirectory(path_tmp + "/" + "img_64fc1.bin");
}

TEST(CvIO, ImageBinaryNonContinuous)
{
  // This test is similar to the binary dump test, but with non-continuous matrix data

  std::string path_tmp = io::getTempDirectoryPath();

  cv::Mat img_8uc4(500, 1000, CV_8UC4, cv::Scalar(125,125,125, 255));

  // Extracting a roi of a matrix will result in non-continuous data
  cv::Mat img_roi = img_8uc4(cv::Rect2i(200, 200, 200, 200));
  img_roi.at<cv::Vec4b>(199, 199) = cv::Vec4b(53, 53, 53, 255);

  EXPECT_EQ(img_roi.isContinuous(), false);

  io::saveImage(img_roi, path_tmp + "/" + "img_8uc4.bin");
  cv::Mat img_roi_copy = io::loadImage(path_tmp + "/" + "img_8uc4.bin");

  EXPECT_EQ(img_roi_copy.at<cv::Vec4b>(199, 199), cv::Vec4b(53, 53, 53, 255));

  io::removeFileOrDirectory(path_tmp + "/" + "img_8uc4.bin");
}

TEST(CvIO, ImageBinaryLegacy)
{
  // This test will check, whether the initial binary specification for cv::Mat have changed or not. We do this by
  // loading a file that was saved in 11/2020. If something was modified since then, the test will show that.
  cv::Mat img_8uc4 = io::loadImage("img_8uc4.bin");
  EXPECT_EQ(img_8uc4.at<cv::Vec4b>(100, 100), cv::Vec4b(125, 125, 125, 255));
}

TEST(CvIO, UnsupportedSuffix)
{
  // Test to show that unsupported file suffix throw an error
  EXPECT_ANY_THROW(io::loadImage("img_8uc4.gif"));
}

TEST(CvIO, FileNotExist)
{
  // Test to show that non-existent files throw an error
  EXPECT_ANY_THROW(io::loadImage("/path/does/not/matter.gif"));
}