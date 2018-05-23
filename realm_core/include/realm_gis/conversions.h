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

#ifndef PROJECT_GIS_CONVERSIONS_H
#define PROJECT_GIS_CONVERSIONS_H

#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>
#include <gdal/cpl_conv.h>

#include <realm_types/utm32.h>
#include <realm_types/wgs84.h>

namespace realm
{
namespace gis
{

/*!
 * @brief Converter for a pose (position and heading only) in WGS84 to UTM coordinate frame
 * @param wgs Pose in WGS84 coordinate frame
 * @return Pose in UTM coordinate frame
 */
UTMPose convertToUTM(const WGSPose &wgs);

/*!
 * @brief Converter for a pose (position and heading only) in UTM to WGS84 coordinate frame
 * @param wgs Pose in UTM coordinate frame
 * @return Pose in WGS84 coordinate frame
 */
WGSPose convertToWGS84(const UTMPose &utm);

}
}

#endif //PROJECT_GIS_CONVERSIONS_H
