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

#ifndef GENERAL_TESTBED_TILE_CACHE_H
#define GENERAL_TESTBED_TILE_CACHE_H

#include <map>
#include <unordered_map>
#include <vector>

#include <opencv2/highgui.hpp>

#include <realm_core/loguru.h>
#include <realm_core/worker_thread_base.h>
#include <realm_io/utilities.h>
#include <realm_ortho/tile.h>

namespace realm
{

class TileCache : public WorkerThreadBase
{
public:
  struct LayerMetaData
  {
    std::string name;
    int type;
    int interpolation_flag;
  };

  struct CacheElement
  {
    using Ptr = std::shared_ptr<CacheElement>;
    long timestamp;
    std::vector<LayerMetaData> layer_meta;
    Tile::Ptr tile;
    bool was_written;

    mutable std::mutex mutex;
  };

  using CacheElementGrid = std::map<int, std::map<int, CacheElement::Ptr>>;

public:
  TileCache(const std::string &id, double sleep_time, bool verbose);
  ~TileCache();

  void add(int zoom_level, const std::vector<Tile::Ptr> &tiles, const cv::Rect2i &roi_idx);

  Tile::Ptr get(int tx, int ty, int zoom_level);

  void setOutputFolder(const std::string &dir);

  void flushAll();
  void loadAll();

//private:

  bool _has_init_directories;

  std::mutex _mutex_settings;
  std::string _dir_toplevel;

  std::mutex _mutex_cache;
  std::map<int, CacheElementGrid> _cache;

  std::mutex _mutex_do_update;
  bool _do_update;

  std::mutex _mutex_roi_prev_request;
  std::map<int, cv::Rect2i> _roi_prev_request;

  std::mutex _mutex_roi_prediction;
  std::map<int, cv::Rect2i> _roi_prediction;

  bool process() override;

  void reset() override;

  void load(const CacheElement::Ptr &element) const;
  void write(const CacheElement::Ptr &element) const;

  void writePNG(const cv::Mat &data, const std::string &filepath) const;
  void writeBinary(const cv::Mat &data, const std::string &filepath) const;

  cv::Mat readPNG(const std::string &filepath) const;
  cv::Mat readBinary(const std::string &filepath) const;

  void flush(const CacheElement::Ptr &element) const;

  bool isCached(const CacheElement::Ptr &element) const;

  size_t estimateByteSize(const Tile::Ptr &tile) const;

  void updatePrediction(int zoom_level, const cv::Rect2i &roi_current);

  void createDirectories(const std::string &toplevel, const std::vector<std::string> &layer_names, const std::string &tile_tree);

};

}

#endif //GENERAL_TESTBED_TILE_CACHE_H
