

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

public:
  TileCache(const std::string &id, double sleep_time, const std::string &output_directory, bool verbose);
  ~TileCache();

  void add(int zoom_level, const std::vector<Tile::Ptr> &tiles, const cv::Rect2i &roi_idx);

  Tile::Ptr get(int tx, int ty, int zoom_level);

  void setOutputFolder(const std::string &dir);

  void flushAll();
  void loadAll();

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

  bool process() override;

  void reset() override;

  void load(const CacheElement::Ptr &element) const;
  void write(const CacheElement::Ptr &element) const;

  void flush(const CacheElement::Ptr &element) const;

  bool isCached(const CacheElement::Ptr &element) const;

  size_t estimateByteSize(const Tile::Ptr &tile) const;

  void updatePrediction(int zoom_level, const cv::Rect2i &roi_current);

  void createDirectories(const std::string &toplevel, const std::vector<std::string> &layer_names, const std::string &tile_tree);

};

}

#endif //GENERAL_TESTBED_TILE_CACHE_H
