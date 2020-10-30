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

#ifndef GENERAL_TESTBED_GDAL_WARPER_H
#define GENERAL_TESTBED_GDAL_WARPER_H

#include <realm_io/gis_export.h>

#include <gdal/gdal_priv.h>
#include <gdal/gdalwarper.h>
#include <gdal/ogr_spatialref.h>
#include <gdal/cpl_conv.h>

namespace realm
{
namespace gis
{

class GdalWarper
{
public:
  GdalWarper();

  void setTargetEPSG(int epsg_code);

  /*!
   * @brief: According to: https://gdal.org/tutorials/warp_tut.html
   * @param map Map to be warped into a new coordinate system. Must not contain more than one layer.
   * @param zone UTM zone of the grid. TODO: should probably be solved smarter in the future
   * @return Warped map in the desired coordinate system. Coordinate system must be cartesian.
   */
  CvGridMap::Ptr warpMap(const CvGridMap &map, uint8_t zone);

  // TBD
  void warpPoints();

private:

  int _epsg_target;

  GDALDriver* _driver;

  /*!
   * @brief For some reason GDAL sets the no data value to 0.0 for floating point matrices, even though all options
   * are set to NaN. So for now this is a quickfix finding 0.0 values and setting them to NaN. Not ideal, because 0.0
   * can also be a valid point.
   * @param data Floating point matrix to be fixed
   */
  void fixGdalNoData(cv::Mat &data);
};

} // namespace gis
} // namespace realm

#endif //GENERAL_TESTBED_GDAL_WARPER_H
