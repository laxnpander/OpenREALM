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

#include <realm_vslam_base/visual_slam_settings_factory.h>
#include <realm_io/utilities.h>

using namespace realm;

VisualSlamSettings::Ptr VisualSlamSettingsFactory::load(const std::string &filepath, const std::string &directory)
{
  std::string method = VisualSlamSettings::sneakParameterFromFile<std::string>("type", filepath);
  if (method == "ORB_SLAM3")
    return loadOrbSlam2(filepath, directory);
  if (method == "OPEN_VSLAM")
    return loadOpenVslam(filepath, directory);
  if (method == "SVO")
    return loadDefault<SvoSettings>(filepath, directory);
  if (method == "SVO2")
    return loadDefault<Svo2Settings>(filepath, directory);
  if (method == "DSO")
    return loadDefault<DsoSettings>(filepath, directory);
  throw (std::invalid_argument("Error: Loading visual slam settings failed. Method '" + method + "' not recognized"));
}

VisualSlamSettings::Ptr VisualSlamSettingsFactory::loadOrbSlam2(const std::string &filepath, const std::string &directory)
{
  auto settings = std::make_shared<OrbSlamSettings>();
  settings->loadFromFile(filepath);

  // Check and correct paths
  settings->set("path_vocabulary", directory + "/orb_slam3/ORBvoc.bin");
  return std::move(settings);
}

VisualSlamSettings::Ptr VisualSlamSettingsFactory::loadOpenVslam(const std::string &filepath, const std::string &directory)
{
  auto settings = std::make_shared<OpenVslamSettings>();
  settings->loadFromFile(filepath);

  // Check and correct paths
  settings->set("path_vocabulary", directory + "/orb_slam3/ORBvoc.bin");
  return std::move(settings);
}

template <typename T>
VisualSlamSettings::Ptr VisualSlamSettingsFactory::loadDefault(const std::string &filepath, const std::string &directory)
{
  auto settings = std::make_shared<T>();
  settings->loadFromFile(filepath);
  return settings;
}