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

#ifndef PROJECT_DENSIFIER_SETTINGS_FACTORY_H
#define PROJECT_DENSIFIER_SETTINGS_FACTORY_H

#include <realm_densifier_base/densifier_settings.h>

namespace realm
{

class DensifierSettingsFactory
{
  public:
    /*!
     * @brief Loads densifier settings from a .yaml file. According to the parameter "type: FRAMEWORK" the correct
     *        settings are selected. Depending on the settings afterwards the correct densification framework is loaded.
     * @param filepath Absolute path of the settings .yaml file
     * @param directory Directory of the settings file. Is only necessary if the loaded framework needs additional
     *        files provided relative to the settings file
     * @return Densifier settings file
     */
    static DensifierSettings::Ptr load(const std::string &filepath,
                                       const std::string &directory);
  private:
    /*!
     * @brief Default loading function for standard settings. Can be specialized if more complex loading has to be performed.
     * @tparam T Type of the settings, see all derived classes from "DensifierSettings"
     * @param settings Densifier base class, is only needed to call "sneakParameter(...)" to get the "type" parameter
     * @param filepath Absolute path of the settings file
     * @param directory Directory of the settings file (optional)
     * @return Densifier settings according to the template provided
     */
    template<typename T>
    static DensifierSettings::Ptr loadDefault(DensifierSettings::Ptr settings,
                                              const std::string &filepath,
                                              const std::string &directory);
};

} // namespace realm

#endif //PROJECT_DENSIFIER_SETTINGS_FACTORY_H
