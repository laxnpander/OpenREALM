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

#include <realm_core/camera_settings_factory.h>

#include "test_helper.h"

// gtest
#include <gtest/gtest.h>

using namespace realm;

TEST(Settings, Access)
{
  auto settings = std::make_shared<DummySettings>();

  EXPECT_EQ((*settings)["parameter_int"].toInt(), 5);
  EXPECT_EQ((*settings)["parameter_double"].toInt(), 2);
  EXPECT_EQ((*settings)["parameter_string"].toString(), "dummy_string");
  EXPECT_NEAR((*settings)["parameter_double"].toDouble(), 2.4, 10e-6);

  EXPECT_EQ(settings->has("parameter_int"), true);
  EXPECT_EQ(settings->has("parameter_float"), false);
  EXPECT_THROW((*settings)["parameter_float"], std::out_of_range);
}

