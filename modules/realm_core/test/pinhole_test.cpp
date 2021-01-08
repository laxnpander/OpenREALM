

#include <iostream>
#include <realm_core/camera.h>
#include <realm_core/camera_settings_factory.h>

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

  EXPECT_NEAR(copy.cx(), K.at<double>(0, 2), 10e-6);
  EXPECT_NEAR(copy.cy(), K.at<double>(1, 2), 10e-6);
  EXPECT_NEAR(copy.fx(), K.at<double>(0, 0), 10e-6);
  EXPECT_NEAR(copy.fy(), K.at<double>(1, 1), 10e-6);
  EXPECT_NEAR(copy.width(), width, 10e-6);
  EXPECT_NEAR(copy.height(), height, 10e-6);
  EXPECT_NEAR(copy.k1(), distortion.at<double>(0), 10e-6);
  EXPECT_NEAR(copy.k2(), distortion.at<double>(1), 10e-6);
  EXPECT_NEAR(copy.p1(), distortion.at<double>(2), 10e-6);
  EXPECT_NEAR(copy.p2(), distortion.at<double>(3), 10e-6);
  EXPECT_NEAR(copy.hasDistortion(), cam.hasDistortion(), 10e-6);
  EXPECT_NEAR(copy.R().at<double>(1, 1), cam.R().at<double>(1, 1), 10e-6);
  EXPECT_NEAR(copy.t().at<double>(1), cam.t().at<double>(1), 10e-6);
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

  EXPECT_NEAR(copy.cx(), K.at<double>(0, 2), 10e-6);
  EXPECT_NEAR(copy.cy(), K.at<double>(1, 2), 10e-6);
  EXPECT_NEAR(copy.fx(), K.at<double>(0, 0), 10e-6);
  EXPECT_NEAR(copy.fy(), K.at<double>(1, 1), 10e-6);
  EXPECT_NEAR(copy.width(), width, 10e-6);
  EXPECT_NEAR(copy.height(), height, 10e-6);
  EXPECT_NEAR(copy.k1(), distortion.at<double>(0), 10e-6);
  EXPECT_NEAR(copy.k2(), distortion.at<double>(1), 10e-6);
  EXPECT_NEAR(copy.p1(), distortion.at<double>(2), 10e-6);
  EXPECT_NEAR(copy.p2(), distortion.at<double>(3), 10e-6);
  EXPECT_NEAR(copy.hasDistortion(), cam.hasDistortion(), 10e-6);
  EXPECT_NEAR(copy.R().at<double>(1, 1), cam.R().at<double>(1, 1), 10e-6);
  EXPECT_NEAR(copy.t().at<double>(1), cam.t().at<double>(1), 10e-6);
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

  EXPECT_NEAR(p1.at<double>(0)/p1.at<double>(3), cam_origin_in_world.at<double>(0), 10e-6);
  EXPECT_NEAR(p1.at<double>(1)/p1.at<double>(3), cam_origin_in_world.at<double>(1), 10e-6);
  EXPECT_NEAR(p1.at<double>(2)/p1.at<double>(3), cam_origin_in_world.at<double>(2), 10e-6);
  EXPECT_NEAR(p2.at<double>(0)/p2.at<double>(3), cam_origin_in_cam.at<double>(0), 10e-6);
  EXPECT_NEAR(p2.at<double>(1)/p2.at<double>(3), cam_origin_in_cam.at<double>(1), 10e-6);
  EXPECT_NEAR(p2.at<double>(2)/p2.at<double>(3), cam_origin_in_cam.at<double>(2), 10e-6);
}

TEST(Pinhole, Resize)
{
  // For this test we create a pinhole camera, resize it to fit e.g. a smaller image and see if it lines up with our
  // expectations
  Pinhole cam = createDummyPinhole();
  Pinhole cam_resized = cam.resize(0.5);

  EXPECT_NEAR(0.5 * cam.fx(), cam_resized.fx(), 10e-6);
  EXPECT_NEAR(0.5 * cam.fy(), cam_resized.fy(), 10e-6);
  EXPECT_NEAR(0.5 * cam.cx(), cam_resized.cx(), 10e-6);
  EXPECT_NEAR(0.5 * cam.cy(), cam_resized.cy(), 10e-6);
  EXPECT_NEAR(0.5 * cam.width(), cam_resized.width(), 10e-6);
  EXPECT_NEAR(0.5 * cam.height(), cam_resized.height(), 10e-6);
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

  EXPECT_NEAR(roi.x, 0.0, 10e-6);
  EXPECT_NEAR(roi.y, 0.0, 10e-6);
  EXPECT_NEAR(roi.width, cam.height(), 10e-6);
  EXPECT_NEAR(roi.height, cam.width(), 10e-6);

  // Testing smaller projection functions as well. Here we project the image corner (0, 0) with the rotated camera
  // into the world frame. The pose of the camera was chosen in such manner, that this projection is crossing the world
  // origin in (0/0/0)
  cv::Mat p1 = cam.projectPointToWorld(0, 0, 1200);

  EXPECT_NEAR(p1.at<double>(0), 0.0, 10e-6);
  EXPECT_NEAR(p1.at<double>(1), 0.0, 10e-6);
  EXPECT_NEAR(p1.at<double>(2), 0.0, 10e-6);
}

TEST(Pinhole, Settings)
{
  auto settings = CameraSettingsFactory::load("calib.yaml");

  EXPECT_EQ((*settings)["type"].toString(), "pinhole");
  EXPECT_NEAR((*settings)["fps"].toDouble(), 10.0, 10e-6);
  EXPECT_EQ((*settings)["width"].toInt(), 1200);
  EXPECT_EQ((*settings)["height"].toInt(), 1000);
  EXPECT_NEAR((*settings)["fx"].toDouble(), 1200.0, 10e-6);
  EXPECT_NEAR((*settings)["fy"].toDouble(), 1200.0, 10e-6);
  EXPECT_NEAR((*settings)["cx"].toDouble(), 600.0, 10e-6);
  EXPECT_NEAR((*settings)["cy"].toDouble(), 500.0, 10e-6);
  EXPECT_NEAR((*settings)["k1"].toDouble(), 0.1, 10e-6);
  EXPECT_NEAR((*settings)["k2"].toDouble(), 0.2, 10e-6);
  EXPECT_NEAR((*settings)["p1"].toDouble(), 0.3, 10e-6);
  EXPECT_NEAR((*settings)["p2"].toDouble(), 0.4, 10e-6);
  EXPECT_NEAR((*settings)["k3"].toDouble(), 0.5, 10e-6);

  EXPECT_ANY_THROW((*settings)["Some non-existent param"].toDouble());
  EXPECT_ANY_THROW(auto settings_wrong = CameraSettingsFactory::load("wrong.yaml"));
  EXPECT_ANY_THROW(auto settings_unsupported = CameraSettingsFactory::load("unsupported.yaml"));
}