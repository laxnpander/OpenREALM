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

#ifndef PROJECT_ENUMS_H
#define PROJECT_ENUMS_H

namespace realm
{

enum class SurfaceAssumption
{
    PLANAR,
    ELEVATION
};

/*!
 * @brief Enumerations for matrix overwrite functionalities.
 * @var REALM_OVERWRITE_ALL Input matrix overwrites all elements of the destination matrix that it overlaps with
 * @var REALM_OVERWRITE_ZERO Input matrix overwrites all zero elements of the destination matrix that it overlaps with
 * @var REALM_OVERWRITE_WITH_NON_ZERO Input matrix overwrites only, if it contains non zero values
 * @var REALM_OVERWRITE_NONE Input matrix overwrites nothing
 */
enum
{
    REALM_OVERWRITE_ALL,
    REALM_OVERWRITE_ZERO,
    REALM_OVERWRITE_WITH_NON_ZERO,
    REALM_OVERWRITE_NONE
};

} // namespace realm

#endif //PROJECT_ENUMS_H
