

#include <iostream>
#include <realm_core/camera.h>
#include <realm_core/stereo.h>

#include "test_helper.h"

// gtest
#include <gtest/gtest.h>

using namespace realm;
using namespace camera;

TEST(Stereo, ReprojectDepthMap)
{
  // For this test we create an artificial camera and depthmap, reproject the latter into the world frame and check if
  // the coordinates are as expected
  auto cam = std::make_shared<Pinhole>(createDummyPinhole());
  cam->setPose(createDummyPose()); // note that the dummy pose is not trivial

  cv::Mat depthmap(cam->height(), cam->width(), CV_32F);

  // Lets create an artificial depth map, which represents a plane with 45° tilt. So basically picture this:
  // We create a plane in distance 1200, which is the distance of the camera to the xy-plane. Then we tilt it making the
  // depth values range on the far left of the depth map to 1200+600 (depth = 1800), and to 1200-600 (600) on the far right.
  for (int r = 0; r < depthmap.rows; ++r)
    for (int c = 0; c < depthmap.cols; ++c)
      depthmap.at<float>(r, c) = 1200.0 + 600.0 - static_cast<float>(c);

  cv::Mat img3d = stereo::reprojectDepthMap(cam, depthmap);

  // Get the corners to check if they have the correct 3d space position
  cv::Vec3d ulc = img3d.at<cv::Vec3d>(0, 0);
  cv::Vec3d urc = img3d.at<cv::Vec3d>(0, img3d.cols-1);
  cv::Vec3d lrc = img3d.at<cv::Vec3d>(img3d.rows-1, img3d.cols-1);
  cv::Vec3d llc = img3d.at<cv::Vec3d>(img3d.rows-1, 0);

  // Also get the center of the image where the depth is exactly 1200 and the 3D point should be a z=0
  cv::Vec3d center = img3d.at<cv::Vec3d>(img3d.rows/2, img3d.cols/2);

  // Check the z-coordinate of the edges. Note that on the right side at max column the depth is -1 because the image
  // has an even number of width
  EXPECT_DOUBLE_EQ(ulc[2], -600.0);
  EXPECT_DOUBLE_EQ(urc[2], +599.0);
  EXPECT_DOUBLE_EQ(lrc[2], +599.0);
  EXPECT_DOUBLE_EQ(llc[2], -600.0);
  EXPECT_DOUBLE_EQ(center[2], 0.0);
}

TEST(Stereo, DepthMapFromPointCloud)
{
  // This test is basically the same as the one prior, just in reverse direction. At first we create the point cloud,
  // and then we project it into a depth map. Note that the point cloud representation is not as img3d, but as matrix
  // with row(i) = point_i, therefore col(0) -> x-coordinates, col(1) -> y-coordinates, col(2) -> z-coordinates.
  auto cam = std::make_shared<Pinhole>(createDummyPinhole());
  cam->setPose(createDummyPose()); // note that the dummy pose is not trivial

  // Here we resize the camera before using it, as it is difficult to create a dense enough point cloud for testing purposes
  // without having round issues interferring.
  auto cam_resized = std::make_shared<Pinhole>(cam->resize(0.1));

  cv::Mat points = (cv::Mat_<double>(4, 3) <<
          -250.0, -300, -600.0,
          250.0, 895, 600.0,
          745.0, 895.0, 600.0,
          1245, -300.0, -600.0);

  cv::Mat depthmap = stereo::computeDepthMapFromPointCloud(cam_resized, points);

  EXPECT_FLOAT_EQ(depthmap.at<float>(0, 0), 1800.0);
  EXPECT_FLOAT_EQ(depthmap.at<float>(0, depthmap.cols-1), 600.0);
  EXPECT_FLOAT_EQ(depthmap.at<float>(depthmap.rows-1, depthmap.cols-1), 600.0);
  EXPECT_FLOAT_EQ(depthmap.at<float>(depthmap.rows-1, 0), 1800.0);
}

TEST(Stereo, NormalsFromDepthMap)
{
  // For this test we create an artificial camera and depthmap and compute the normals for all pixels
  auto cam = std::make_shared<Pinhole>(createDummyPinhole());
  cam->setPose(createDummyPose());

  cv::Mat depthmap(cam->height(), cam->width(), CV_32F);

  // We create the same artificial depth map as in prior tests, which represents a plane with 45° tilt. So basically picture this:
  // The plane is in distance 1200, which is the distance of the camera to the xy-plane. Then we tilt it making the
  // depth values range on the far left of the depth map to 1200+600 (depth = 1800), and to 1200-600 (600) on the far right.
  for (int r = 0; r < depthmap.rows; ++r)
    for (int c = 0; c < depthmap.cols; ++c)
      depthmap.at<float>(r, c) = 1200.0 + 600.0 - static_cast<float>(c);

  cv::Mat normal_map = stereo::computeNormalsFromDepthMap(depthmap);

  cv::Vec3f ulc = normal_map.at<cv::Vec3f>(0, 0);
  cv::Vec3f urc = normal_map.at<cv::Vec3f>(0, normal_map.cols-1);
  cv::Vec3f lrc = normal_map.at<cv::Vec3f>(normal_map.rows-1, normal_map.cols-1);
  cv::Vec3f llc = normal_map.at<cv::Vec3f>(normal_map.rows-1, 0);

  // Normals should have 45° angle, check the corners again
  EXPECT_NEAR(fabs(acos(ulc[2]/cv::norm(ulc))*180/3.1415), 45.0, 10e-2);
  EXPECT_NEAR(fabs(acos(urc[2]/cv::norm(urc))*180/3.1415), 45.0, 10e-2);
  EXPECT_NEAR(fabs(acos(lrc[2]/cv::norm(lrc))*180/3.1415), 45.0, 10e-2);
  EXPECT_NEAR(fabs(acos(llc[2]/cv::norm(llc))*180/3.1415), 45.0, 10e-2);
}

TEST(Stereo, BaselineFromPose)
{
  cv::Mat p1 = cv::Mat::eye(3, 4, CV_64F);
  p1.at<double>(0, 3) = 350;
  p1.at<double>(1, 3) = 600;
  p1.at<double>(2, 3) = 120;

  cv::Mat p2 = cv::Mat::eye(3, 4, CV_64F);
  p2.at<double>(0, 3) = 800;
  p2.at<double>(1, 3) = 100;
  p2.at<double>(2, 3) = 450;

  double baseline = stereo::computeBaselineFromPose(p1, p2);

  EXPECT_NEAR(baseline, 749.266, 0.01);
}