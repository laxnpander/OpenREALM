
#include <iostream>

#include <realm_io/mvs_export.h>
#include <realm_io/utilities.h>

// gtest
#include <gtest/gtest.h>

using namespace realm;

TEST(MvsIO, LoadMvs)
{
  io::MvsInterface interface;
  io::MvsArchive::SerializeLoad(interface, "/home/alex/Documents/output/21-03-19_17-40-50/mosaicing/mvs/data.mvs");

  io::MvsInterface::Platform platform = interface.platforms.front();
  std::cout << platform.cameras.front().name << std::endl;
  std::cout << platform.cameras.front().K << std::endl;
  std::cout << platform.poses.size() << std::endl;
  std::cout << platform.poses.front().R << std::endl;
  for (const auto& i : interface.images)
  {
    std::cout << i.name << std::endl;
  }
}