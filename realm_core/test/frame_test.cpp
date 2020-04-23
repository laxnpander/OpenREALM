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
#include <realm_core/frame.h>

#include "test_helper.h"

// gtest
#include <gtest/gtest.h>

using namespace realm;

TEST(Frame, Container)
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

  EXPECT_EQ(frame->getCameraId(), camera_id);
  EXPECT_EQ(frame->getFrameId(), frame_id);
  EXPECT_EQ(frame->getTimestamp(), timestamp);
  EXPECT_EQ(frame->getImageRaw().at<uchar>(30, 30), img.at<uchar>(30, 30));
  EXPECT_EQ(frame->getCamera()->width(), 1200.0);
  EXPECT_EQ(frame->getResizedCamera()->width(), 600.0);
  EXPECT_EQ(frame->getGnssUtm().easting, 603976);
  EXPECT_EQ(frame->hasAccuratePose(), true);
  EXPECT_EQ(frame->isKeyframe(), true);
  EXPECT_EQ(frame->isImageResizeSet(), true);
  EXPECT_EQ(frame->getSurfaceAssumption(), SurfaceAssumption::PLANAR);
}

TEST(Frame, AddSurfacePoints)
{
  // For this test we check if setting the surface points in the frame message works as intended. Because the frame
  // internally computes values like the observed scene depth (min, max, median), this complexer than setting a member
  // variable. As for the procedure: We create a dummy frame, some observed points and compare the computed scene depth
  // to a value that we expect.
  Frame::Ptr frame = createDummyFrame();

  // We set a dummy pose for the dummy pinhole model
  frame->setVisualPose(createDummyPose());

  // Create some artificial point cloud
  cv::Mat surface_points = (cv::Mat_<double>(3, 3) << 0, 32, 0, 15, 50, -130, 2, 2, 50);

  // The camera is facing straight down in -z at position t = (x, y, z) in the world frame, so the created surface points
  // are equally spread around the x-y-plane so their median is exactly at depth = z
  frame->setSurfacePoints(surface_points);

  EXPECT_EQ(frame->getMedianSceneDepth(), frame->getCamera()->t().at<double>(2));
  EXPECT_EQ(frame->getMinSceneDepth(), frame->getCamera()->t().at<double>(2)-50);
  EXPECT_EQ(frame->getMaxSceneDepth(), frame->getCamera()->t().at<double>(2)+130);
}

TEST(Frame, Georeference)
{
  // For this test we apply a fake georeference to the frame. The general concept is, that at first the frame only
  // holds parameters in an arbitrary visual coordinate system including observed surface points. It's only that after
  // computing a georeference all spatial data (pose, surface points) is transferred to a global UTM coordinate system.
  Frame::Ptr frame = createDummyFrame();

  // We set a dummy pose for the dummy pinhole model
  frame->setVisualPose(createDummyPose());

  // Create some artificial point cloud
  cv::Mat surface_points = (cv::Mat_<double>(3, 3) << 0, 32, 0, 15, 50, -1200, 2, 2, 600);
  frame->setSurfacePoints(surface_points);

  // Create some artifical transformation into the UTM frame.
  cv::Mat T_georeference = cv::Mat::eye(4, 4, CV_64F);

  // We assume our scale in the visual coordinate frame is bigger than the real world scale (usually it is the other way
  // around, but our values for the dummy pose are quite big in this example). So at first we scale it down, so that our
  // camera is moving in 100m altitude:
  T_georeference.at<double>(0, 0) = 1/12.0f;
  T_georeference.at<double>(1, 1) = 1/12.0f;
  T_georeference.at<double>(2, 2) = 1/12.0f;

  // Next we offset it 50m higher and to an UTM area in Braunschweig, Germany :)
  T_georeference.at<double>(0, 3) = 603976;
  T_georeference.at<double>(1, 3) = 5791569;
  T_georeference.at<double>(2, 3) = 50;

  // We don't do any rotation for now, might be better for completeness in the future. So now apply the transformation:
  frame->applyGeoreference(T_georeference);

  // This should have put our pose at 150m altitude and our scene depth at min = 50, median = 100, max = 200
  // Note, that transformation to the world frame involves huge scaling most of the times. This will cause rounding errors
  EXPECT_NEAR(frame->getMinSceneDepth(), 50, 10e-3);
  EXPECT_NEAR(frame->getMaxSceneDepth(), 200, 10e-3);
  EXPECT_NEAR(frame->getMedianSceneDepth(), 100, 10e-3);
  EXPECT_NEAR(frame->getPose().at<double>(2, 3), 150, 10e-3);
}