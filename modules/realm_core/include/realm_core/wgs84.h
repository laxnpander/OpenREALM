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

#ifndef PROJECT_WGS84_H
#define PROJECT_WGS84_H

/*!
 * @brief Class for WGS84 coordinates. Currently we avoid GDAL dependencies in the realm_core library, therefore no
 *        conversion is provided here. That might change in the future.
 */

class WGSPose
{
  public:
    double latitude;
    double longitude;
    double altitude;
    double heading;
};

#endif //PROJECT_WGS84_H
