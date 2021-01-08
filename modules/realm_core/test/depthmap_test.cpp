

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