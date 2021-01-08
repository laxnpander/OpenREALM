

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

