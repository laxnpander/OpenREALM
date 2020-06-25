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

struct GDALDatasetMeta
{
  GDALDataType datatype;
  uint8_t zone;
  double geoinfo[6];
};

/*!
 * @brief Interface function to save GeoTIFFs from CvGridMap
 * @param map CvGridMap containing the image data and geo information in UTM coordinates
 * @param color_layer_name Name of the layer to be rendered
 * @param zone UTM zone of the map ROI
 * @param filename Full name of the image to be saved. In case of split save the name will be modified adding channel information
 * @param do_build_overview Flag to build internal overviews with GDAL before the .tif is translated
 * @param do_split_save Flag to save all channels separately. The filename is modified in this case to represent the channel
 * information also, e.g. ortho -> ortho_b, ortho_g, ortho_r, ortho_a
 * @param gdal_profile Collection of options that are passed to the GDAL driver
 */
void saveGeoTIFF(const CvGridMap &map,
                 const std::string &color_layer_name,
                 const uint8_t &zone,
                 const std::string &filename,
                 bool do_build_overview = false,
                 bool do_split_save = false,
                 GDALProfile gdal_profile = GDALProfile::COG);

/*!
 * @brief Internal function to save GeoTIFFs. Usually the user should not be forced to compute the GDALDatasetMeta beforehand.
 * @param data OpenCV matrix data, can be multi or single layered
 * @param meta Meta informations about the dataset
 * @param filename Full filename of the resulting .tif file
 * @param do_build_overview Flag to build internal overviews with GDAL before the .tif is translated
 * @param gdal_profile Collection of options that are passed to the GDAL driver
 */
void saveGeoTIFF(const cv::Mat &data,
                 const GDALDatasetMeta &meta,
                 const std::string &filename,
                 bool do_build_overview,
                 GDALProfile gdal_profile);

/*!
 * @brief Internal function to translate CvGridMap information to GDAL meta data.
 * @param map CvGridMap which should be saved as GeoTIFF
 * @param color_layer_name Name of the layer inside the grid map to be saved
 * @param zone UTM zone of the geo position
 * @return GDAL meta information about the dataset
 */
GDALDatasetMeta* computeGDALDatasetMeta(const CvGridMap &map, const std::string &color_layer_name, uint8_t zone);

/*!
 * @brief
 * @param gdal_profile
 * @return
 */
char** getExportOptionsGeoTIFF(GDALProfile gdal_profile);

} // namespace io
} // namespace realm

#endif //PROJECT_GEOEXPORT_H