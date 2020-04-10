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

#include <realm_densifier_base/densifier_settings_factory.h>

using namespace realm;

DensifierSettings::Ptr DensifierSettingsFactory::load(const std::string &filepath, const std::string &directory)
{
  auto settings = std::make_shared<DensifierSettings>();
  std::string method = settings->sneakParamFromFile<std::string>("type", filepath);
  if (method == "DUMMY")
    return loadDefault<DensifierDummySettings>(settings, filepath, directory);
  if (method == "PSL")
    return loadDefault<PlaneSweepSettings>(settings, filepath, directory);
//if (method == "YOUR_IMPLEMENTATION")
//  return loadDefault<YOUR_IMPLEMENTATION_SETTINGS>(settings, directory, filename);
  throw (std::invalid_argument("Error: Loading densifier settings failed. Method '" + method + "' not recognized"));
}

template <typename T>
DensifierSettings::Ptr DensifierSettingsFactory::loadDefault(DensifierSettings::Ptr settings,
                                                             const std::string &filepath,
                                                             const std::string &directory)
{
  settings = std::make_shared<T>();
  settings->loadFromFile(filepath);
  return settings;
}