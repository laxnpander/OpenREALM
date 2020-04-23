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
#include <realm_core/camera.h>

#include "test_helper.h"

// gtest
#include <gtest/gtest.h>

using namespace realm;
using namespace camera;

TEST(Pinhole, CopyConstructor)
{
  // We create the camera properties manually here and don't use the dummies to avoid a possible dependency on the
  // assignment operator. Otherwise if the assignment operator is corrupted and we set cam = createDummyPinhole(...)
  // we might not see a problem, even though there is one.
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
  distortion.at<double>(1) = 0.1;
  distortion.at<double>(2) = 0.2;
  distortion.at<double>(3) = 0.3;
  distortion.at<double>(4) = 0.4;

  Pinhole cam(K, distortion, width, height);
  cam.setPose(createDummyPose());
  Pinhole copy(cam);

  EXPECT_EQ(copy.cx(), K.at<double>(0, 2));
  EXPECT_EQ(copy.cy(), K.at<double>(1, 2));
  EXPECT_EQ(copy.fx(), K.at<double>(0, 0));
  EXPECT_EQ(copy.fy(), K.at<double>(1, 1));
  EXPECT_EQ(copy.width(), width);
  EXPECT_EQ(copy.height(), height);
  EXPECT_EQ(copy.k1(), distortion.at<double>(0));
  EXPECT_EQ(copy.k2(), distortion.at<double>(1));
  EXPECT_EQ(copy.p1(), distortion.at<double>(2));
  EXPECT_EQ(copy.p2(), distortion.at<double>(3));
  EXPECT_EQ(copy.hasDistortion(), cam.hasDistortion());
  EXPECT_EQ(copy.R().at<double>(1, 1), cam.R().at<double>(1, 1));
  EXPECT_EQ(copy.t().at<double>(1), cam.t().at<double>(1));
}

TEST(Pinhole, CopyAssign)
{
  // We create the camera properties manually here and don't use the dummies to avoid a possible dependency on the
  // assignment operator. Otherwise if the assignment operator is corrupted and we set cam = createDummyPinhole(...)
  // we might not see a problem, even though there is one.
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
  distortion.at<double>(1) = 0.1;
  distortion.at<double>(2) = 0.2;
  distortion.at<double>(3) = 0.3;
  distortion.at<double>(4) = 0.4;

  Pinhole cam(K, distortion, width, height);
  cam.setPose(createDummyPose());
  Pinhole copy = cam;

  EXPECT_EQ(copy.cx(), K.at<double>(0, 2));
  EXPECT_EQ(copy.cy(), K.at<double>(1, 2));
  EXPECT_EQ(copy.fx(), K.at<double>(0, 0));
  EXPECT_EQ(copy.fy(), K.at<double>(1, 1));
  EXPECT_EQ(copy.width(), width);
  EXPECT_EQ(copy.height(), height);
  EXPECT_EQ(copy.k1(), distortion.at<double>(0));
  EXPECT_EQ(copy.k2(), distortion.at<double>(1));
  EXPECT_EQ(copy.p1(), distortion.at<double>(2));
  EXPECT_EQ(copy.p2(), distortion.at<double>(3));
  EXPECT_EQ(copy.hasDistortion(), cam.hasDistortion());
  EXPECT_EQ(copy.R().at<double>(1, 1), cam.R().at<double>(1, 1));
  EXPECT_EQ(copy.t().at<double>(1), cam.t().at<double>(1));
}

TEST(Pinhole, Transformations)
{
  // For this test we look if the transformation definitions inside the pinhole camera are not corrupted. T_c2w should
  // transform the camera origin (0, 0, 0) into the world frame ((dummy_pose_x, dummy_pose_y, dummy_pose_z) for the dummy pose).
  // Same goes the other way around.
  Pinhole cam = createDummyPinhole();
  cv::Mat pose = createDummyPose();
  cam.setPose(pose);

  cv::Mat cam_origin_in_cam = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1.0);
  cv::Mat cam_origin_in_world = (cv::Mat_<double>(4, 1) << pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3), 1.0);

  cv::Mat p1 = cam.Tc2w() * cam_origin_in_cam;    // expect: p1 = cam_origin_in_world
  cv::Mat p2 = cam.Tw2c() * cam_origin_in_world;  // expect: p2 = cam_origin_in_cam

  EXPECT_EQ(p1.at<double>(0)/p1.at<double>(3), cam_origin_in_world.at<double>(0));
  EXPECT_EQ(p1.at<double>(1)/p1.at<double>(3), cam_origin_in_world.at<double>(1));
  EXPECT_EQ(p1.at<double>(2)/p1.at<double>(3), cam_origin_in_world.at<double>(2));
  EXPECT_EQ(p2.at<double>(0)/p2.at<double>(3), cam_origin_in_cam.at<double>(0));
  EXPECT_EQ(p2.at<double>(1)/p2.at<double>(3), cam_origin_in_cam.at<double>(1));
  EXPECT_EQ(p2.at<double>(2)/p2.at<double>(3), cam_origin_in_cam.at<double>(2));
}

TEST(Pinhole, Resize)
{
  // For this test we create a pinhole camera, resize it to fit e.g. a smaller image and see if it lines up with our
  // expectations
  Pinhole cam = createDummyPinhole();
  Pinhole cam_resized = cam.resize(0.5);

  EXPECT_EQ(0.5 * cam.fx(), cam_resized.fx());
  EXPECT_EQ(0.5 * cam.fy(), cam_resized.fy());
  EXPECT_EQ(0.5 * cam.cx(), cam_resized.cx());
  EXPECT_EQ(0.5 * cam.cy(), cam_resized.cy());
  EXPECT_EQ(0.5 * cam.width(), cam_resized.width());
  EXPECT_EQ(0.5 * cam.height(), cam_resized.height());
}

TEST(Pinhole, Projections)
{
  // For this test we project the image boundaries into a reference plane. This checks the validity of 5 different
  // projection functions of pinhole camera at once:
  // - computeImageBounds2Ddistorted (however, note: with dummy calibration no distortion is assumed)
  // - computeImageBounds2D: (however, note: with dummy calibration no distortion is assumed)
  // - projectImageBoundsToPlane: As the name suggests, image boundaries are projected into a plane, returns locations
  // - projectImageBoundsToPlaneRoi: Image boundaries are projected into a plane, returns region of interest
  Pinhole cam = createDummyPinhole();

  // We create a dummy pose that is designed in such way, that the projection into the x/y-plane of the world produces
  // a region of interest at (x = 0, y = 0) with size (width = image height, height = image width)
  cam.setPose(createDummyPose());

  // Create the x/y-plane
  cv::Mat p = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  cv::Mat n = (cv::Mat_<double>(3, 1) << 0, 0, 1.0);

  cv::Rect2d roi = cam.projectImageBoundsToPlaneRoi(p, n);

  EXPECT_EQ(roi.x, 0.0);
  EXPECT_EQ(roi.y, 0.0);
  EXPECT_EQ(roi.width, cam.height());
  EXPECT_EQ(roi.height, cam.width());

  // Testing smaller projection functions as well. Here we project the image corner (width, 0) with the rotated camera
  // into the world frame. The pose of the camera was chosen in such manner, that this projection is crossing the world
  // origin in (0/0/0)
  cv::Mat p1 = cam.projectPointToWorld(cam.width(), 0, 1200);

  EXPECT_EQ(p1.at<double>(0), 0.0);
  EXPECT_EQ(p1.at<double>(1), 0.0);
  EXPECT_EQ(p1.at<double>(2), 0.0);
}