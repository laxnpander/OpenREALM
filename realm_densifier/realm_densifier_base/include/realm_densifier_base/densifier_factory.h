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

#ifndef PROJECT_DENSIFIER_FACTORY_H
#define PROJECT_DENSIFIER_FACTORY_H

#include <opencv2/core.hpp>

#include <realm_densifier_base/densifier_IF.h>
#include <realm_densifier_base/densifier_settings.h>
#include <realm_densifier_base/densifier_dummy.h>

#ifdef USE_CUDA
  #include <realm_densifier_base/plane_sweep.h>
#endif

namespace realm
{
namespace densifier
{

class DensifierFactory
{
  public:
    /*!
     * @brief Factory class for densification frameworks. Depending on the settings file provided, the appropriate
     *        framework interface is loaded. Therefore implementing a custom densifier the following steps are needed:
     *        1) Implement interface class derived from "DensifierIF" for your framework
     *        2) Implement settings class derived from "DensifierSettings
     *        3) Provide a .yaml file with at least "type: YOUR_FRAMEWORK"
     *        4) Register your settings and interface in densifier settings factory and this factory class
     * @param settings Custom settings for densifier framework to be loaded. Most important parameter: type for loading
     *        of the correct framework
     * @return Instantiation of your densification framework
     */
    static DensifierIF::Ptr create(const DensifierSettings::Ptr &settings);
};

} // namespace densifier
} // namespace realm

#endif //PROJECT_DENSIFIER_FACTORY_H
