

#ifndef GENERAL_TESTBED_MAP_TILER_H
#define GENERAL_TESTBED_MAP_TILER_H

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
 * @brief MapTiler allows to slice a geographically referenced CvGridMap in EPSG:3857 coordinates into equally sized
 * tiles on different zoom levels following the Tile Map Service (TMS) specification.
 */
class MapTiler
{
public:
  using Ptr = std::shared_ptr<MapTiler>;

  struct TiledMap
  {
    cv::Rect2i roi;
    std::vector<Tile::Ptr> tiles;
  };

public:

  /*!
   * @brief Besides member initialization the required lookup tables to map zoom level to image resolution are created.
   * @param verbosity Flag to set verbose output
   */
  explicit MapTiler(bool verbosity);

  ~MapTiler() = default;
  MapTiler(const MapTiler &other) = default;

  /*!
   * @brief Slicing the input map in EPSG:3857 coordinates into equally sized tiles on different zoom levels. If no
   * specific zoom levels are provided, only the maximum possible zoom level is tiled. This in turn is computed based on
   * the resolution of the input grid map. We perform an up-scaling to the next higher resolution, so we are not missing
   * out on information provided by the raw data, while accepting higher computational load for interpolated data.
   * @param map Map in EPSG:3857 coordinates
   * @param zoom_level_min (optional) Minimum zoom level that should be tiled. If none provided, only maximum zoom level
   * is created.
   * @param zoom_level_max (optional) Maximum zoom level that should be tiled. If none provided, the maximum zoom level
   * will be automatically computed.
   * @return Tiled map for each requested zoom level. The tiled map consists of a region of interest spanning the
   * coordinates of the tiles (x, y, width, height) on the specific zoom level and the corresponding data as a vector.
   */
  std::map<int, TiledMap> createTiles(const CvGridMap::Ptr &map, int zoom_level_min = -1, int zoom_level_max = -1);

  double getResolution(int zoom_level);

private:

  /// Flag to set verbose output
  int m_verbosity;

  /// Global minimum zoom level
  int m_zoom_level_min;

  /// Global maximum zoom level
  int m_zoom_level_max;

  /// Shift of the coordinate frame origin
  double m_origin_shift;

  /// Size of the tiles in [pix], usually this is 256
  int m_tile_size;

  /// Lookup table to map zoom levels to a specific resolution in [m/pix]
  std::map<int, double> m_lookup_resolution_from_zoom;

  /// Lookup table to map zoom levels to an absolute number of tiles
  std::map<int, long>   m_lookup_nrof_tiles_from_zoom;

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
};

} // namespace realm

#endif //GENERAL_TESTBED_MAP_TILER_H
