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

#include <realm_types/camera_settings_factory.h>

using namespace realm;

CameraSettings::Ptr CameraSettingsFactory::load(const std::string &filepath)
{
  auto settings = std::make_shared<CameraSettings>();
  // Identify camera model
  std::string model_type = settings->sneakParamFromFile<std::string>("type", filepath);
  if (model_type == "pinhole")
    return load<PinholeSettings>(settings, filepath);
  else
    throw(std::out_of_range("Error! Camera type '" + model_type + "' not recognized."));
}

template <typename T>
CameraSettings::Ptr CameraSettingsFactory::load(CameraSettings::Ptr settings,
                                                const std::string &filepath)
{
  // Read from settings file
  settings = std::make_shared<T>();
  settings->loadFromFile(filepath);
  return std::move(settings);
}