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

#ifndef PROJECT_VISUAL_SLAM_SETTINGS_FACTORY_H
#define PROJECT_VISUAL_SLAM_SETTINGS_FACTORY_H

#include <realm_vslam_base/visual_slam_settings.h>

namespace realm
{

class VisualSlamSettingsFactory
{
  public:
    /*!
     * @brief Function to create visual slam settings from .yaml file
     * @param filepath Absolute path to the settings yaml file
     * @param directory Absolute path to the settings directory (used for possible additional files needed to be included)
     * @return Loaded visual slam settings
     */
    static VisualSlamSettings::Ptr load(const std::string &filepath,
                                        const std::string &directory);
  private:
    static VisualSlamSettings::Ptr loadOrbSlam2(const std::string &filepath, const std::string &directory);
    static VisualSlamSettings::Ptr loadOpenVslam(const std::string &filepath, const std::string &directory);
    template <typename T>
    static VisualSlamSettings::Ptr loadDefault(const std::string &filepath, const std::string &directory);
};

} // namespace realm

#endif //PROJECT_VISUAL_SLAM_SETTINGS_FACTORY_H
