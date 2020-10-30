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

#ifndef GENERAL_TESTBED_MAP_TILER_H
#define GENERAL_TESTBED_MAP_TILER_H

#include <iostream>
#include <cmath>
#include <map>

#include <realm_ortho/rectification.h>
#include <realm_ortho/gdal_warper.h>
#include <realm_ortho/tile_cache.h>
#include <realm_core/conversions.h>
#include <realm_core/loguru.h>

namespace realm
{

// NOTE: Many of the utility functions are direct translations of Klokan Petr Pridal's Python implementation
// 'gdal2tiles.py' in https://github.com/OSGeo/gdal/blob/master/gdal/swig/python/scripts/gdal2tiles.py.

/*!
 * @brief This class allows to tile and maintain an existing
 */
class MapTiler
{
public:
  using Ptr = std::shared_ptr<MapTiler>;

  using BlendingFunc = std::function<Tile::Ptr (Tile::Ptr &, Tile::Ptr &)>;

public:

  MapTiler(const std::string &id, const std::string &directory, const std::vector<std::string> &full_layers, bool verbosity);
  ~MapTiler();

  void createTiles(const CvGridMap::Ptr &map, uint8_t zone);

  void registerBlendingFunction(const BlendingFunc &func);

  double getResolution(int zoom_level);

private:

  int _verbosity;

  int _zoom_level_min;
  int _zoom_level_max;

  double _origin_shift;

  /// Size of the tiles in [pix], usually this is 256
  int _tile_size;

  std::string _output_directory;

  std::vector<std::string> _layers_all_zoom_levels;

  std::map<int, double> _lookup_resolution_from_zoom;
  std::map<int, long>   _lookup_nrof_tiles_from_zoom;

  BlendingFunc _blending_merge;
  BlendingFunc _blending_fuse;

  /// Warper to transform incoming grid maps from UTM coordinates to Web Mercator (EPSG:3857)
  gis::GdalWarper _warper;

  TileCache _tile_cache;

  Tile::Ptr merge(const Tile::Ptr &t1, const Tile::Ptr &t2) const;
  Tile::Ptr fuse(const Tile::Ptr &t1, const Tile::Ptr &t2) const;

  void computeTileing(const CvGridMap::Ptr &map, const cv::Rect2d &roi, int zoom_level_min, int zoom_level_max, const BlendingFunc &functor);

  /*!
   * @brief Computes the slippy tile index for a given zoom level that contains the requested coordinate in WGS84. The
   * specifications are documented: https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Pseudo-code
   * @param lat Latitude in WGS84
   * @param lon Longitude in WGS84
   * @param zoom_level Zoom level 0 - 35
   * @return Tile coordinate according to slippy tile standard
   */
  cv::Point2i computeTileFromLatLon(double lat, double lon, int zoom_level) const;

  /*!
   * @brief Each tile is indexed with (tx, ty). Together with the corresponding tile size this coordinate can be
   * transformed into a global pixel coordinate with tx * tile size, ty * tile size. If the resolution is known in
   * [m/pix] we can furthermore transform each pixel coordinate into a meter coordinate.
   * This function converts the pixel coordinates back to meters.
   * @param px Global x-coordinate in pixels
   * @param py Global y-coordinate in pixels
   * @param zoom_level Zoom level of the tile map
   * @return Global coordinate in meters (mx, my)
   */
  cv::Point2d computeMetersFromPixels(int px, int py, int zoom_level);

  /*!
   * @brief Each tile is indexed with (tx, ty). Together with the corresponding tile size this coordinate can be
   * transformed into a global pixel coordinate with tx * tile size, ty * tile size. If the resolution is known in
   * [m/pix] we can furthermore transform each pixel coordinate into a meter coordinate.
   * This function converts the meter coordinates back to pixels.
   * @param mx Global x-coordinates in meters
   * @param my Global y-coordinates in meters
   * @param zoom_level Zoom level of the tile map
   * @return Global coordinate in pixels (px, py)
   */
  cv::Point2i computePixelsFromMeters(double mx, double my, int zoom_level);

  /*!
   * @brief Each tile is indexed with (tx, ty). Together with the corresponding tile size this coordinate can be
   * transformed into a global pixel coordinate with tx * tile size, ty * tile size. If the resolution is known in
   * [m/pix] wgis::GdalWarper warper;e can furthermore transform each pixel coordinate into a meter coordinate.
   * This function returns the tile index from the global pixel coordinates.
   * @param px Global x-coordinate in pixels
   * @param py Global y-coordinate in pixels
   * @param zoom_level Zoom level of the tile map
   * @return Index of the tile corresponding to the global pixel coordinate
   */
  cv::Point2i computeTileFromPixels(int px, int py, int zoom_level);

  /*!
   * @brief Each tile is indexed with (tx, ty). Together with the corresponding tile size this coordinate can be
   * transformed into a global pixel coordinate with tx * tile size, ty * tile size. If the resolution is known in
   * [m/pix] we can furthermore transform each pixel coordinate into a meter coordinate.
   * This function returns the tile index from the global meter coordinates.
   * @param mx Global x-coordinates in meters
   * @param my Global y-coordinates in meters
   * @param zoom_level Zoom level of the tile map
   * @return Index of the tile corresponding to the global pixel coordinate
   */
  cv::Point2i computeTileFromMeters(double mx, double my, int zoom_level);

  /*!
   * @brief Computes the boundaries of tile indices including the region of interest in a geographic frame
   * @param roi Region of interest in geographic frame for which the tile ROI should be computed
   * @param zoom_level Zoom level of the tile map
   * @return
   */
  cv::Rect2i computeTileBounds(const cv::Rect2d &roi, int zoom_level);

  /*!
   * @brief Each tile is indexed with (tx, ty). Together with the corresponding tile size this coordinate can be
   * transformed into a global pixel coordinate with tx * tile size, ty * tile size. If the resolution is known in
   * [m/pix] we can furthermore transform each pixel coordinate into a meter coordinate.
   * This function returns the boundaries of a tile in meters.
   * @param tx Global index of tile for specific zoom level
   * @param ty Global index of tile for specific zoom level
   * @param zoom_level Zoom level of the tile map
   * @return Boundaries of the tile in meters with (mx, my, width, height)
   */
  cv::Rect2d computeTileBoundsMeters(int tx, int ty, int zoom_level);

  /*!
   * @brief Each tile is indexed with (tx, ty). Together with the corresponding tile size this coordinate can be
   * transformed into a global pixel coordinate with tx * tile size, ty * tile size. If the resolution is known in
   * [m/pix] we can furthermore transform each pixel coordinate into a meter coordinate.
   * This function returns the boundaries of a tile in meters.
   * @param idx_roi Boundaries of a region of interest in tile indices
   * @param zoom_level Zoom level of the tile map
   * @return Boundaries of the tile in meters with (mx, my, width, height)
   */
  cv::Rect2d computeTileBoundsMeters(const cv::Rect2i &idx_roi, int zoom_level);

  /*!
   * @brief Computes one lat-lon coordinate for a given tile. It represents the upper left corner of the tile.
   * @param x Coordinate of tile in x-direction
   * @param y Coordinate of tile in y-direction
   * @param zoom_level Zoon level of the map
   * @return (longitude, latitude) of upper left tile corner
   */
  WGSPose computeLatLonForTile(int x, int y, int zoom_level) const;

  /*!
   * @brief Maximal scale down zoom of the pyramid closest to the pixelSize.
   * @param GSD Ground sampling distance in m/pix
   * @param do_upscale By default we perform an upscale to the next smaller resolution, so we can benefit from the
   * maximum detail provided by the input data. With no upscale the next bigger resolution is chosen and some details
   * might get lost while resizing.
   * @return Zoom level
   */
  int computeZoomForPixelSize(double GSD, bool do_upscale = true) const;

  /*!
   * @brief Resolution (meters/pixel) for given zoom level. Latitude can be provided to get a more accurate estimate.
   * Source for formulas: https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
   * @param zoom_level Zoom level for which the resolution (GSD) should be computed
   * @param latitude Rough latitude for area of operation [in degrees]. Provide 0.0 if computation at equator is sufficient.
   * @return resolution of the pixels
   */
  double computeZoomResolution(int zoom_level, double latitude) const;

  /*!
   * @brief Computes the lookup table for the resolution depending on the zoom level. Latitude can be additionally provided
   * to make a more precise estimate.
   * @param latitude (optional) Rough latitude of the area of operation [in degrees]
   */
  void computeLookupResolutionFromZoom(double latitude = 0.0);

  void generateLeaflet();
};

} // namespace realm

#endif //GENERAL_TESTBED_MAP_TILER_H
