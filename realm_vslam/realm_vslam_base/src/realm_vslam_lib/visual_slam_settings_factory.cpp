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

#include <realm_vslam/visual_slam_settings_factory.h>

using namespace realm;

VisualSlamSettings::Ptr VisualSlamSettingsFactory::load(const std::string &filepath, const std::string &directory)
{
  auto settings = std::make_shared<VisualSlamSettings>();
  std::string method = settings->sneakParamFromFile<std::string>("type", filepath);
  if (method == "ORB_SLAM2")
    return loadOrbSlam2(settings, filepath, directory);
  if (method == "SVO")
    return loadDefault<SvoSettings>(settings, filepath, directory);
  if (method == "SVO2")
    return loadDefault<Svo2Settings>(settings, filepath, directory);
  if (method == "DSO")
    return loadDefault<DsoSettings>(settings, filepath, directory);
  throw (std::invalid_argument("Error: Loading visual slam settings failed. Method '" + method + "' not recognized"));
}

VisualSlamSettings::Ptr VisualSlamSettingsFactory::loadOrbSlam2(VisualSlamSettings::Ptr settings,
                                                               const std::string &filepath,
                                                               const std::string &directory)
{
  settings = std::make_shared<OrbSlamSettings>();
  settings->loadFromFile(filepath);

  // Check and correct paths
  settings->set("path_vocabulary", directory + "/orb_slam2/ORBvoc.bin");
  return std::move(settings);
}

template <typename T>
VisualSlamSettings::Ptr VisualSlamSettingsFactory::loadDefault(VisualSlamSettings::Ptr settings,
                                                               const std::string &filepath,
                                                               const std::string &directory)
{
  settings = std::make_shared<T>();
  settings->loadFromFile(filepath);
  return settings;
}