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

#ifndef PROJECT_GEOEXPORT_H
#define PROJECT_GEOEXPORT_H

#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>
#include <gdal/cpl_conv.h>

#include <realm_core/cv_grid_map.h>
#include <realm_core/wgs84.h>
#include <realm_io/utilities.h>

namespace realm
{
namespace io
{

enum class GDALProfile
{
  COG /// Cloud Optimized GeoTIFF: https://trac.osgeo.org/gdal/wiki/CloudOptimizedGeoTIFF
};

/*!
 * @brief Save function for GeoTIFFs interfaced through a CvGridMap.
 * @param map Observed scene that should be saved as GeoTIFF
 * @param color_layer_name Name of the color layer to be saved in the grid map
 * @param zone UTM zone of the passed position
 * @param directory Directory for saving
 * @param name Filename of the output file
 * @return True if successfull
 */
void saveGeoTIFF(const CvGridMap &map,
                 const std::string &color_layer_name,
                 const uint8_t &zone,
                 const std::string &directory,
                 const std::string &name,
                 GDALProfile gdal_profile = GDALProfile::COG);

/*!
 * @brief Save function for GeoTIFFs interfaced through a CvGridMap with unique name creation based on given id.
 * @param map Observed scene that should be saved as GeoTIFF
 * @param color_layer_name Name of the color layer to be saved in the grid map
 * @param zone UTM zone of the passed position
 * @param directory Directory for saving
 * @param name Filename of the output file
 * @param id Unique ID for saving
 * @return True if successfull
 */
void saveGeoTIFF(const CvGridMap &map,
                 const std::string &color_layer_name,
                 const uint8_t &zone,
                 const std::string &directory,
                 const std::string &name,
                 uint32_t id,
                 GDALProfile gdal_profile = GDALProfile::COG);

/*!
 * @brief Save function for GeoTIFFs interfaced through a CvGridMap.
 * @param map Observed scene that should be saved as GeoTIFF
 * @param color_layer_name Name of the color layer to be saved in the grid map
 * @param zone UTM zone of the passed position
 * @param name Absolute filename of the output file
 * @return True if successfull
 */
void saveGeoTIFF(const CvGridMap &map,
                 const std::string &color_layer_name,
                 const uint8_t &zone,
                 const std::string &filename,
                 GDALProfile gdal_profile = GDALProfile::COG);

/*!
 * @brief Save function for GeoTIFFs interfaced through raw minimum input.
 * @param img Image data of the observed scene
 * @param filename Filename of the output file including directory
 * @param geoinfo 6x1 raw array of geo informations organized as:
 * (UTM upper left coordinate eastings,
 *  ground sampling distance,
 *  0.0,
 *  UTM upper left coordinate northings,
 *  0.0,
 *  -ground sampling distance)
 * @param zone UTM zone of the passed position
 * @return
 */
void saveGeoTIFF(const cv::Mat &img,
                 const char *filename,
                 double *geoinfo,
                 const uint8_t &zone,
                 GDALProfile gdal_profile);

char** getExportOptionsGeoTIFF(GDALProfile gdal_profile);

} // namespace io
} // namespace realm

#endif //PROJECT_GEOEXPORT_H