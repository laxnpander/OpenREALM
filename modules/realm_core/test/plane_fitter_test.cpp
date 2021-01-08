

#include <iostream>
#include <realm_core/plane_fitter.h>

#include "test_helper.h"

// gtest
#include <gtest/gtest.h>

using namespace realm;
using Pf = PlaneFitter;

TEST(PlaneFitter, Exact)
{
  // For this test we check if the plane fitter is able to compute a plane exactly with the minimum required points (n = 3).
  // As example we create a plane, whose centroid is in (0, 0, 0) with a normal of 45° rotated around the x-axis.
  std::vector<Pf::Point> points{Pf::Point{0.0, 1.0, 1.0}, Pf::Point{1.0, -0.5, -0.5}, Pf::Point{-1.0, -0.5, -0.5}};

  PlaneFitter plane_fitter;
  Pf::Plane plane = plane_fitter.estimate(points);

  double r = sqrt(plane.n.x*plane.n.x + plane.n.y*plane.n.y + plane.n.z*plane.n.z);
  double phi = atan(plane.n.y/plane.n.z) * 180 / 3.1415;

  EXPECT_DOUBLE_EQ(plane.pt.x, 0.0);
  EXPECT_DOUBLE_EQ(plane.pt.y, 0.0);
  EXPECT_DOUBLE_EQ(plane.pt.z, 0.0);
  EXPECT_DOUBLE_EQ(r, 1.0);
  EXPECT_NEAR(fabs(phi), 45.0, 10e-2); // result can either be positive or negative
}

TEST(PlaneFitter, BestFit)
{
  // After checking that the exact solution is working, we test if the best fit is also giving results as expected.
  // Therefore we now need more than 3 points. To make it easier to follow this time the plane normal is equal to the
  // z-coordinate axis.
  std::vector<Pf::Point> points{
          Pf::Point{-2.0, -1.0, -1.0},
          Pf::Point{2.0, 1.0, -1.0},
          Pf::Point{1.0, 5.0, 1.0},
          Pf::Point{-1.0, -5.0, 1.0},
          Pf::Point{0.0, 0.0, 0.0}
  };

  PlaneFitter plane_fitter;
  Pf::Plane plane = plane_fitter.estimate(points);

  double r = sqrt(plane.n.x*plane.n.x + plane.n.y*plane.n.y + plane.n.z*plane.n.z);
  double phi = atan(plane.n.y/plane.n.z) * 180 / 3.1415;

  EXPECT_DOUBLE_EQ(plane.pt.x, 0.0);
  EXPECT_DOUBLE_EQ(plane.pt.y, 0.0);
  EXPECT_DOUBLE_EQ(plane.pt.z, 0.0);
  EXPECT_DOUBLE_EQ(r, 1.0);
  EXPECT_NEAR(fabs(phi), 0.0, 10e-2);
}