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

#include <realm_ortho/map_tiler.h>

#include <unordered_map>

using namespace realm;

MapTiler::MapTiler(const std::string &id, const std::string &directory)
    : _verbosity(1),
      _zoom_level_min(11),
      _zoom_level_max(35),
      _tile_size(256),
      _output_directory(directory),
      _tile_cache(id, 10, false)
{
  _origin_shift = 2 * M_PI * 6378137 / 2.0;

  // Setup lookup table for zoom level resolution
  for (int i = 0; i < _zoom_level_max; ++i)
  {
    _lookup_nrof_tiles_from_zoom[i] = std::pow(2, i);
  }

  computeLookupResolutionFromZoom();

  _warper.setTargetEPSG(3857);

  _tile_cache.setOutputFolder(_output_directory);

  _tile_cache.start();
}

MapTiler::~MapTiler()
{
  _tile_cache.requestFinish();
  _tile_cache.join();
}

double MapTiler::getResolution(int zoom_level)
{
  auto it = _lookup_resolution_from_zoom.find(zoom_level);
  if (it != _lookup_resolution_from_zoom.end())
    return it->second;
  else
    throw(std::invalid_argument("Error getting resolution for zoom level: Lookup table does not contain key!"));
}

void MapTiler::createTiles(const CvGridMap::Ptr &map, uint8_t zone)
{
  // Transform CvGridMap to Web Mercator (EPSG:3857)
  CvGridMap::Ptr map_3857 = _warper.warpMap(*map, zone);

  // Set the region of interest before entering the loop, so that even though we transform the map itself the original
  // boundaries will matter
  cv::Rect2d roi = map_3857->roi();

  // Map was transformed. Now we have to adjust the resolution to fit web style tiles
  int zoom_level_base = computeZoomForPixelSize(map_3857->resolution());

  for (int zoom_level = zoom_level_base; zoom_level >= _zoom_level_min; --zoom_level)
  {
    double zoom_resolution = getResolution(zoom_level);

    LOG_IF_F(INFO, _verbosity, "Tileing map on zoom level %i, resolution = %4.4f", zoom_level, zoom_resolution);

    map_3857->changeResolution(zoom_resolution);

    // Map is now in the right coordinate frame and has the correct resolution. It's time to start tileing it
    // Therefore first identify how many tiles we have to split our map into by computing the tile indices
    cv::Rect2i tile_bounds_idx = computeTileBounds(roi, zoom_level);

    // With the tile indices we can compute the exact region of interest in the geographic frame in meters
    cv::Rect2d tile_bounds_meters = computeTileBoundsMeters(tile_bounds_idx, zoom_level);

    // Because our map is not yet guaranteed to have exactly the size of the tile region, we have to perform padding to
    // to fit exactly the tile map boundaries
    map_3857->extendToInclude(tile_bounds_meters);

    // Now our map has exactly the size of the desired tile region and can be tiled accordingly
    cv::Mat data = (*map_3857)["data"];

    std::vector<Tile::Ptr> tiles;
    for (int x = 0; x < tile_bounds_idx.width; ++x)
      // Note: Coordinate system of the tiles is up positive, while image is down positive. Therefore the inverse loop
      for (int y = tile_bounds_idx.height; y > 0; --y)
      {
        cv::Rect2i data_roi(x*256, y*256, 256, 256);
        Tile::Ptr tile_current = std::make_shared<Tile>(zoom_level, tile_bounds_idx.x + x, tile_bounds_idx.y + tile_bounds_idx.height - y, data(data_roi));
        Tile::Ptr tile_cached = _tile_cache.get(tile_bounds_idx.x + x, tile_bounds_idx.y + tile_bounds_idx.height - y, zoom_level);

        Tile::Ptr tile_output = nullptr;
        if (tile_cached)
        {
          tile_output = blend(tile_current, tile_cached);
          tile_cached->unlock();
        }
        else
        {
          tile_output = tile_current;
        }
        tiles.push_back(tile_output);
      }

    _tile_cache.add(zoom_level, tiles, tile_bounds_idx);

    _tile_cache.updatePrediction(zoom_level, tile_bounds_idx);

  }
}

Tile::Ptr MapTiler::blend(const Tile::Ptr &t1, const Tile::Ptr &t2)
{
  if (t2->data().empty())
    throw(std::runtime_error("Error: Tile data is empty. Likely a multi threading problem!"));

  cv::Mat bgra[4];
  split(t1->data(), bgra);

  cv::Mat mask = (bgra[3] < 255);
  t2->data().copyTo(t1->data(), mask);

  return t1;
}

void MapTiler::computeLookupResolutionFromZoom(double latitude)
{
  for (int i = 0; i < _zoom_level_max; ++i)
  {
    _lookup_resolution_from_zoom[i] = computeZoomResolution(i, latitude);
  }
}

cv::Point2i MapTiler::computeTileFromLatLon(double lat, double lon, int zoom_level) const
{
  double lat_rad = lat * M_PI / 180.0;
  auto n = static_cast<double>(_lookup_nrof_tiles_from_zoom.at(zoom_level));

  cv::Point2i pos;
  pos.x = static_cast<int>(std::floor((lon + 180.0) / 360.0 * n));
  pos.y = static_cast<int>(std::floor((1.0 - asinh(tan(lat_rad)) / M_PI) / 2.0 * n));
  return pos;
}

cv::Point2d MapTiler::computeMetersFromPixels(int px, int py, int zoom_level)
{
  cv::Point2d meters;
  double resolution = _lookup_resolution_from_zoom.at(zoom_level);
  meters.x = px * resolution - _origin_shift;
  meters.y = py * resolution - _origin_shift;
  return meters;
}

cv::Point2i MapTiler::computePixelsFromMeters(double mx, double my, int zoom_level)
{
  cv::Point2i pixels;
  double resolution = _lookup_resolution_from_zoom.at(zoom_level);
  pixels.x = (mx + _origin_shift) / resolution;
  pixels.y = (my + _origin_shift) / resolution;
  return pixels;
}

cv::Point2i MapTiler::computeTileFromPixels(int px, int py, int zoom_level)
{
  cv::Point2i tile;
  tile.x = int(std::ceil(px / (double)(_tile_size)) - 1);
  tile.y = int(std::ceil(py / (double)(_tile_size)) - 1);
  return tile;
}

cv::Point2i MapTiler::computeTileFromMeters(double mx, double my, int zoom_level)
{
  cv::Point2i pixels = computePixelsFromMeters(mx, my, zoom_level);
  return computeTileFromPixels(pixels.x, pixels.y, zoom_level);
}

cv::Rect2i MapTiler::computeTileBounds(const cv::Rect2d &roi, int zoom_level)
{
  cv::Point2i tile_idx_low = computeTileFromMeters(roi.x, roi.y, zoom_level);
  cv::Point2i tile_idx_high = computeTileFromMeters(roi.x + roi.width, roi.y + roi.height, zoom_level);
  // Note: +1 in both directions because tile origin sits in the lower left corner of the tile
  return cv::Rect2i(tile_idx_low.x, tile_idx_low.y, tile_idx_high.x - tile_idx_low.x + 1, tile_idx_high.y - tile_idx_low.y + 1);
}

cv::Rect2d MapTiler::computeTileBoundsMeters(int tx, int ty, int zoom_level)
{
  cv::Point2d p_min = computeMetersFromPixels(tx * _tile_size, ty * _tile_size, zoom_level);
  cv::Point2d p_max = computeMetersFromPixels((tx + 1) * _tile_size, (ty + 1) * _tile_size, zoom_level);
  return cv::Rect2d(p_min.x, p_min.y, p_max.x - p_min.x, p_max.y - p_min.y);
}

cv::Rect2d MapTiler::computeTileBoundsMeters(const cv::Rect2i &idx_roi, int zoom_level)
{
  cv::Rect2d tile_bounds_low = computeTileBoundsMeters(idx_roi.x, idx_roi.y, zoom_level);
  cv::Rect2d tile_bounds_high = computeTileBoundsMeters(idx_roi.x + idx_roi.width + 1, idx_roi.y + idx_roi.height + 1, zoom_level);
  return cv::Rect2d(tile_bounds_low.x, tile_bounds_low.y, tile_bounds_high.x - tile_bounds_low.x, tile_bounds_high.y - tile_bounds_low.y);
}

WGSPose MapTiler::computeLatLonForTile(int x, int y, int zoom_level) const
{
  auto n = static_cast<double>(_lookup_nrof_tiles_from_zoom.at(zoom_level));
  double k = M_PI - 2.0 * M_PI * y / n;

  WGSPose wgs{};
  wgs.latitude = 180.0 / M_PI * atan(0.5 * (exp(k) - exp(-k)));
  wgs.longitude = x / n * 360.0 - 180;

  return wgs;
}

int MapTiler::computeZoomForPixelSize(double GSD, bool do_upscale) const
{
  for (int i = 0; i < _zoom_level_max; ++i)
  {
    if (GSD >= _lookup_resolution_from_zoom.at(i) + 10e-3)
    {
      if (do_upscale)
        return std::max(0, i);
      else
        return std::max(0, i - 1);
    }
  }
  return _zoom_level_max-1;
}

double MapTiler::computeZoomResolution(int zoom_level, double latitude) const
{
  if (fabs(latitude) < 10e-3)
    return (2 * M_PI * 6378137 / _tile_size) / _lookup_nrof_tiles_from_zoom.at(zoom_level);
  else
    return 156543.03 * cos(latitude*M_PI/180) / _lookup_nrof_tiles_from_zoom.at(zoom_level);
}

void MapTiler::generateLeaflet()
{
//  std::unordered_map<std::string, std::string> args;
//  args["title"] = "OpenREALM";
//  args["htmltitle"] = self.options.title;
//  args["south"], args['west'], args['north'], args['east'] = self.swne;
//  args["centerlon"] = (args['north'] + args['south']) / 2.;
//  args["centerlat"] = (args['west'] + args['east']) / 2.;
//  args["minzoom"] = "15";
//  args["maxzoom"] = "19";
//  args["beginzoom"] = self.tmaxz;
//  args["tileformat"] = self.tileext;
//  args["copyright"] = self.options.copyright.replace('"', '\\"');
}