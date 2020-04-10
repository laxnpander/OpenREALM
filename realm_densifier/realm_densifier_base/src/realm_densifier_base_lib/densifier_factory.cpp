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

#include <realm_densifier_base/densifier_factory.h>

using namespace realm;

DensifierIF::Ptr densifier::DensifierFactory::create(const DensifierSettings::Ptr &settings)
{
  if (settings->get<std::string>("type") == "DUMMY")
    return std::make_shared<densifier::Dummy>(settings);
#ifdef USE_CUDA
  if (settings->get<std::string>("type") == "PSL")
    return std::make_shared<densifier::PlaneSweep>(settings);
#endif
  throw std::invalid_argument("Error: Densifier framework '" + settings->get<std::string>("type") + "' not found");
}