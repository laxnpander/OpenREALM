

#include <iostream>
#include <realm_core/conversions.h>

// gtest
#include <gtest/gtest.h>

using namespace realm;

TEST(Conversion, UTM_WGS)
{
  // This is a very simple test. We create a UTM pose, convert it into WGS84 and vice versa.
  UTMPose utm(604347, 5792556, 100.0, 23.0, 32, 'U');
  WGSPose wgs = gis::convertToWGS84(utm);

  EXPECT_NEAR(wgs.latitude, 52.273462, 10e-6);
  EXPECT_NEAR(wgs.longitude, 10.529350, 10e-6);

  UTMPose utm2 = gis::convertToUTM(wgs);

  EXPECT_NEAR(utm2.easting, utm.easting, 10e-6);
  EXPECT_NEAR(utm2.northing, utm.northing, 10e-6);
  EXPECT_NEAR(utm2.altitude, utm.altitude, 10e-6);
  EXPECT_NEAR(utm2.heading, utm.heading, 10e-6);
  EXPECT_EQ(utm2.zone, utm.zone);
  EXPECT_EQ(utm2.band, utm.band);
}