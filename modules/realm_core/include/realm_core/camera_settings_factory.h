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

#ifndef PROJECT_CAMERA_SETTINGS_FACTORY_H
#define PROJECT_CAMERA_SETTINGS_FACTORY_H

#include <realm_core/camera_settings.h>

namespace realm
{

class CameraSettingsFactory
{
  public:
    /*!
     * @brief Function for creation of camera settings by loading them from a .yaml file. See also camera_settings.h
     *        for further informations about what parameters must be inside the .yaml files.
     * @param filepath Absolute path to the file to be loaded
     * @return Shared pointer to the camera settings, that were loaded.
     * @throws out_of_range Gets thrown, if type of camera settings are requested that do not exist (currently everything
     *         except pinhole camera
     */
    static CameraSettings::Ptr load(const std::string &filepath);

  private:
    /*!
     * @brief Templated private function for creation of camera settings by loading them from a .yaml file. See also
     *        camera_settings.h for further informations about what parameters must be inside the .yaml files. Is used
     *        by default for the factory
     * @tparam T Explicit camera settings type, e.g. PinholeSettings
     * @param filepath Absolute path to settings file
     * @return Explicit camera settings of type T
     */
    template <typename T>
    static CameraSettings::Ptr load(const std::string &filepath);
};

} // namespace realm

#endif //PROJECT_CAMERA_SETTINGS_FACTORY_H
