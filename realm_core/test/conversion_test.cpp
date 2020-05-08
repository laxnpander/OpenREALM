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