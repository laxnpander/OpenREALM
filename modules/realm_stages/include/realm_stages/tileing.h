/**
* This file is part of OpenREALM.
*
* Copyright (C) 2020 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
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

#ifndef PROJECT_TILEING_H
#define PROJECT_TILEING_H

#include <deque>
#include <chrono>
#include <map>
#include <unordered_map>
#include <vector>
#include <opencv2/highgui.hpp>

#include <realm_stages/stage_base.h>
#include <realm_stages/conversions.h>
#include <realm_stages/stage_settings.h>
#include <realm_core/frame.h>
#include <realm_core/cv_grid_map.h>
#include <realm_core/analysis.h>
#include <realm_io/cv_export.h>
#include <realm_io/gis_export.h>
#include <realm_io/utilities.h>
#include <realm_ortho/map_tiler.h>
#include <realm_core/worker_thread_base.h>
#include <realm_ortho/tile.h>

namespace realm
{
namespace stages
{

class TileCache;

class Tileing : public StageBase
{
  public:
    using Ptr = std::shared_ptr<Tileing>;
    using ConstPtr = std::shared_ptr<const Tileing>;
    friend TileCache;

    struct SaveSettings
    {

    };

  public:
    explicit Tileing(const StageSettings::Ptr &stage_set, double rate);
    ~Tileing();

    void initStagePath(std::string stage_path);
    void initStagePath(std::string stage_path, std::string cache_path);
    void addFrame(const Frame::Ptr &frame) override;
    bool process() override;
    void saveAll();

    void deleteCache();
    void deleteCache(std::string layer);

  private:
    std::deque<Frame::Ptr> m_buffer;
    std::mutex m_mutex_buffer;

    SaveSettings m_settings_save;

    UTMPose::Ptr m_utm_reference;

    /// If true, uses the TMS standard for y (bottom-top) rather than the Google/OSM standard (top-bottom)
    bool m_generate_tms_tiles;

    /// The minimum zoom level to generate
    int m_min_tile_zoom;

    /// The maximum zoom to generate.  May not be generated if GSD isn't sufficient
    int m_max_tile_zoom;

    /// Indicates we should wipe the cache directory when starting or resetting the stage
    bool m_delete_cache_on_init;

    /// If true, files from disk will be loaded into the tile cache before stitching begins
    bool m_load_cache_on_init;

    /// Indicates we have published that initial tile cache update for our cache_on_init load
    bool m_initial_cache_published;

    /// The directory to store the output map tiles in, defaults to log directory
    std::string m_cache_path;

    /// Warper to transform incoming grid maps from UTM coordinates to Web Mercator (EPSG:3857)
    gis::GdalWarper m_warper;

    MapTiler::Ptr m_map_tiler;
    std::unique_ptr<TileCache> m_tile_cache;

    Tile::Ptr merge(const Tile::Ptr &t1, const Tile::Ptr &t2);
    Tile::Ptr blend(const Tile::Ptr &t1, const Tile::Ptr &t2);

    void finishCallback() override;
    void printSettingsToLog() override;

    void reset() override;
    void initStageCallback() override;
    uint32_t getQueueDepth() override;

    void publish(const Frame::Ptr &frame, TileCache &cache, std::map<int, cv::Rect2i> updated_tiles, uint64_t timestamp);

    void saveIter(uint32_t id, const CvGridMap::Ptr &map_update);
    Frame::Ptr getNewFrame();
};

  class TileCache : public WorkerThreadBase
  {
  public:
    using Ptr = std::shared_ptr<TileCache>;

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
    using CacheElementItem = std::map<int, CacheElement::Ptr>;

  public:
    TileCache(Tileing *tiling_stage, double sleep_time, std::string output_directory, bool verbose);
    ~TileCache();

    void add(int zoom_level, const std::vector<Tile::Ptr> &tiles, const cv::Rect2i &roi_idx);

    Tile::Ptr get(int tx, int ty, int zoom_level);

    std::map<int, cv::Rect2i> getBounds() const;

    void setOutputFolder(const std::string &dir);
    std::string getCachePath(const std::string &layer);

    void publishWrittenTiles(std::map<int, cv::Rect2i> &update_region, int tiles_written);

    void flushAll();
    void loadAll();

    void loadDiskCache();
    void deleteCache();
    void deleteCache(std::string layer);

  private:

    bool m_has_init_directories;

    std::mutex m_mutex_settings;
    std::string m_dir_toplevel;

    std::mutex m_mutex_cache;
    std::map<int, CacheElementGrid> m_cache;

    std::mutex m_mutex_do_update;
    bool m_do_update;

    std::mutex m_mutex_roi_prev_request;
    std::map<int, cv::Rect2i> m_roi_prev_request;

    std::mutex m_mutex_roi_prediction;
    std::map<int, cv::Rect2i> m_roi_prediction;

    std::mutex m_mutex_roi_map_bounds;
    std::map<int, cv::Rect2i> m_cache_bounds;

    Tileing *m_tiling_stage;

    bool process() override;

    void reset() override;

    void load(const CacheElement::Ptr &element);
    void write(const CacheElement::Ptr &element);

    void flush(const CacheElement::Ptr &element);

    bool isCached(const CacheElement::Ptr &element) const;

    size_t estimateByteSize(const Tile::Ptr &tile) const;

    void updatePrediction(int zoom_level, const cv::Rect2i &roi_current);

    void createDirectories(const std::string &toplevel, const std::vector<std::string> &layer_names, const std::string &tile_tree);

  };

} // namespace stages
} // namespace realm

#endif //PROJECT_TILEING_H
