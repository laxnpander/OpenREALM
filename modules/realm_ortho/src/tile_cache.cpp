

#include <cstdio>

#include <realm_ortho/tile_cache.h>
#include <realm_io/cv_import.h>
#include <realm_io/cv_export.h>

using namespace realm;

TileCache::TileCache(const std::string &id, double sleep_time, const std::string &output_directory, bool verbose)
 : WorkerThreadBase("tile_cache_" + id, sleep_time, verbose),
   m_dir_toplevel(output_directory),
   m_has_init_directories(false),
   m_do_update(false)
{
  m_data_ready_functor = [=]{ return (m_do_update || isFinishRequested()); };
}

TileCache::~TileCache()
{
  flushAll();
}

void TileCache::setOutputFolder(const std::string &dir)
{
  std::lock_guard<std::mutex> lock(m_mutex_settings);
  m_dir_toplevel = dir;
}

bool TileCache::process()
{
  bool has_processed = false;

  if (m_mutex_do_update.try_lock())
  {
    long t;

    // Give update lock free as fast as possible, so we won't block other threads from adding data
    bool do_update = m_do_update;
    m_do_update = false;
    m_mutex_do_update.unlock();

    if (do_update)
    {
      int n_tiles_written = 0;

      t = getCurrentTimeMilliseconds();

      for (auto &cached_elements_zoom : m_cache)
      {
        cv::Rect2i roi_prediction = m_roi_prediction.at(cached_elements_zoom.first);
        for (auto &cached_elements_column : cached_elements_zoom.second)
        {
          for (auto &cached_elements : cached_elements_column.second)
          {
            std::lock_guard<std::mutex> lock(cached_elements.second->mutex);
            cached_elements.second->tile->lock();

            if (!cached_elements.second->was_written)
            {
              n_tiles_written++;
              write(cached_elements.second);
            }

            if (isCached(cached_elements.second))
            {
              int tx = cached_elements.second->tile->x();
              int ty = cached_elements.second->tile->y();
              if (tx < roi_prediction.x || tx > roi_prediction.x + roi_prediction.width
                  || ty < roi_prediction.y || ty > roi_prediction.y + roi_prediction.height)
              {
                flush(cached_elements.second);
              }
            }
            cached_elements.second->tile->unlock();
          }
        }
      }

      LOG_IF_F(INFO, m_verbose, "Tiles written: %i", n_tiles_written);
      LOG_IF_F(INFO, m_verbose, "Timing [Cache Flush]: %lu ms", getCurrentTimeMilliseconds() - t);

      has_processed = true;
    }
  }
  return has_processed;
}

void TileCache::reset()
{
  m_cache.clear();
}

void TileCache::add(int zoom_level, const std::vector<Tile::Ptr> &tiles, const cv::Rect2i &roi_idx)
{
  std::lock_guard<std::mutex> lock(m_mutex_cache);

  // Assuming all tiles are based on the same data, therefore have the same number of layers and layer names
  std::vector<std::string> layer_names = tiles[0]->data()->getAllLayerNames();

  std::vector<LayerMetaData> layer_meta;
  for (const auto &layer_name : layer_names)
  {
    // Saving the name and the type of the layer into the meta data
    CvGridMap::Layer layer = tiles[0]->data()->getLayer(layer_name);
    layer_meta.emplace_back(LayerMetaData{layer_name, layer.data.type(), layer.interpolation});
  }

  if (!m_has_init_directories)
  {
    createDirectories(m_dir_toplevel + "/", layer_names, "");
    m_has_init_directories = true;
  }

  auto it_zoom = m_cache.find(zoom_level);

  long timestamp = getCurrentTimeMilliseconds();

  long t = getCurrentTimeMilliseconds();

  // Cache for this zoom level already exists
  if (it_zoom != m_cache.end())
  {
    for (const auto &t : tiles)
    {
      // Here we find a tile grid for a specific zoom level and add the new tiles to it.
      // Important: Tiles that already exist will be overwritten!
      t->lock();
      auto it_tile_x = it_zoom->second.find(t->x());
      if (it_tile_x == it_zoom->second.end())
      {
        // Zoom level exists, but tile column is
        createDirectories(m_dir_toplevel + "/", layer_names, "/" + std::to_string(zoom_level) + "/" + std::to_string(t->x()));
        it_zoom->second[t->x()][t->y()].reset(new CacheElement{timestamp, layer_meta, t, false});
      }
      else
      {
        auto it_tile_xy = it_tile_x->second.find(t->y());
        if (it_tile_xy == it_tile_x->second.end())
        {
          // Zoom level and column was found, but tile did not yet exist
          it_tile_x->second[t->y()].reset(new CacheElement{timestamp, layer_meta, t, false});
        }
        else
        {
          // Existing tile was found inside zoom level and column
          it_tile_xy->second->mutex.lock(); // note: mutex goes out of scope after this operation, no unlock needed.
          it_tile_xy->second.reset(new CacheElement{timestamp, layer_meta, t, false});
        }
      }
      t->unlock();
    }
  }
  // Cache for this zoom level does not yet exist
  else
  {
    createDirectories(m_dir_toplevel + "/", layer_names, "/" + std::to_string(zoom_level));

    CacheElementGrid tile_grid;
    for (const auto &t : tiles)
    {
      // By assigning a new grid of tiles to the zoom level we overwrite all existing data. But in this case there was
      // no prior data found for the specific zoom level.
      t->lock();
      auto it_tile_x = it_zoom->second.find(t->x());
      if (it_tile_x == it_zoom->second.end())
        createDirectories(m_dir_toplevel + "/", layer_names, "/" + std::to_string(zoom_level) + "/" + std::to_string(t->x()));

      tile_grid[t->x()][t->y()].reset(new CacheElement{timestamp, layer_meta, t, false});
      t->unlock();
    }
    m_cache[zoom_level] = tile_grid;
  }

  LOG_IF_F(INFO, m_verbose, "Timing [Cache Push]: %lu ms", getCurrentTimeMilliseconds() - t);

  updatePrediction(zoom_level, roi_idx);

  std::lock_guard<std::mutex> lock1(m_mutex_do_update);
  m_do_update = true;
  notify();
}

Tile::Ptr TileCache::get(int tx, int ty, int zoom_level)
{
  auto it_zoom = m_cache.find(zoom_level);
  if (it_zoom == m_cache.end())
  {
    return nullptr;
  }

  auto it_tile_x = it_zoom->second.find(tx);
  if (it_tile_x == it_zoom->second.end())
  {
    return nullptr;
  }

  auto it_tile_xy = it_tile_x->second.find(ty);
  if (it_tile_xy == it_tile_x->second.end())
  {
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(it_tile_xy->second->mutex);

  // Warning: We lock the tile now and return it to the calling thread locked. Therefore the responsibility to unlock
  // it is on the calling thread!
  it_tile_xy->second->tile->lock();
  if (!isCached(it_tile_xy->second))
  {
    load(it_tile_xy->second);
  }

  return it_tile_xy->second->tile;
}

void TileCache::flushAll()
{
  int n_tiles_written = 0;

  LOG_IF_F(INFO, m_verbose, "Flushing all tiles...");

  long t = getCurrentTimeMilliseconds();

  for (auto &zoom_levels : m_cache)
    for (auto &cache_column : zoom_levels.second)
      for (auto &cache_element : cache_column.second)
      {
        std::lock_guard<std::mutex> lock(cache_element.second->mutex);
        cache_element.second->tile->lock();
        if (!cache_element.second->was_written)
        {
          write(cache_element.second);
          n_tiles_written++;
        }

        cache_element.second->tile->data() = nullptr;
        cache_element.second->tile->unlock();
      }

  LOG_IF_F(INFO, m_verbose, "Tiles written: %i", n_tiles_written);
  LOG_IF_F(INFO, m_verbose, "Timing [Flush All]: %lu ms", getCurrentTimeMilliseconds() - t);
}

void TileCache::loadAll()
{
  for (auto &zoom_levels : m_cache)
    for (auto &cache_column : zoom_levels.second)
      for (auto &cache_element : cache_column.second)
      {
        std::lock_guard<std::mutex> lock(cache_element.second->mutex);
        cache_element.second->tile->lock();
        if (!isCached(cache_element.second))
          load(cache_element.second);
        cache_element.second->tile->unlock();
      }
}

void TileCache::load(const CacheElement::Ptr &element) const
{
  for (const auto &meta : element->layer_meta)
  {
    std::string filename = m_dir_toplevel + "/"
                           + meta.name + "/"
                           + std::to_string(element->tile->zoom_level()) + "/"
                           + std::to_string(element->tile->x()) + "/"
                           + std::to_string(element->tile->y());

    int type = meta.type & CV_MAT_DEPTH_MASK;

    switch(type)
    {
      case CV_8U:
        filename += ".png";
        break;
      case CV_16U:
        filename += ".bin";
        break;
      case CV_32F:
        filename += ".bin";
        break;
      case CV_64F:
        filename += ".bin";
        break;
      default:
        throw(std::invalid_argument("Error reading tile: data type unknown!"));
    }

    if (io::fileExists(filename))
    {
      cv::Mat data = io::loadImage(filename);

      element->tile->data()->add(meta.name, data, meta.interpolation_flag);

      LOG_IF_F(INFO, m_verbose, "Read tile from disk: %s", filename.c_str());
    }
    else
    {
      LOG_IF_F(WARNING, m_verbose, "Failed reading tile from disk: %s", filename.c_str());
      throw(std::invalid_argument("Error loading tile."));
    }
  }
}

void TileCache::write(const CacheElement::Ptr &element) const
{
  for (const auto &meta : element->layer_meta)
  {
    cv::Mat data = element->tile->data()->get(meta.name);

    std::string filename = m_dir_toplevel + "/"
                           + meta.name + "/"
                           + std::to_string(element->tile->zoom_level()) + "/"
                           + std::to_string(element->tile->x()) + "/"
                           + std::to_string(element->tile->y());

    int type = data.type() & CV_MAT_DEPTH_MASK;

    switch(type)
    {
      case CV_8U:
        filename += ".png";
        break;
      case CV_16U:
        filename += ".bin";
        break;
      case CV_32F:
        filename += ".bin";
        break;
      case CV_64F:
        filename += ".bin";
        break;
      default:
        throw(std::invalid_argument("Error writing tile: data type unknown!"));
    }

    io::saveImage(data, filename);

    element->was_written = true;
  }
}

void TileCache::flush(const CacheElement::Ptr &element) const
{
  if (!element->was_written)
    write(element);

  for (const auto &meta : element->layer_meta)
  {
    element->tile->data()->remove(meta.name);
  }

  LOG_IF_F(INFO, m_verbose, "Flushed tile (%i, %i, %i) [zoom, x, y]", element->tile->zoom_level(), element->tile->x(), element->tile->y());
}

bool TileCache::isCached(const CacheElement::Ptr &element) const
{
  return !(element->tile->data()->empty());
}

size_t TileCache::estimateByteSize(const Tile::Ptr &tile) const
{
  tile->lock();
  //size_t bytes = tile->data().total() * tile->data().elemSize();
  tile->unlock();

  //return bytes;
  return 0;
}

void TileCache::updatePrediction(int zoom_level, const cv::Rect2i &roi_current)
{
  std::lock_guard<std::mutex> lock(m_mutex_roi_prev_request);
  std::lock_guard<std::mutex> lock1(m_mutex_roi_prediction);

  auto it_roi_prev_request = m_roi_prev_request.find(zoom_level);
  if (it_roi_prev_request == m_roi_prev_request.end())
  {
    // There was no previous request, so there can be no prediction which region of tiles might be needed in the next
    // processing step. Therefore set the current roi to be the prediction for the next request.
    m_roi_prediction[zoom_level] = roi_current;
  }
  else
  {
    // We have a previous roi that was requested, therefore we can extrapolate what the next request might look like
    // utilizing our current roi
    auto it_roi_prediction = m_roi_prediction.find(zoom_level);
    it_roi_prediction->second.x = roi_current.x + (roi_current.x - it_roi_prev_request->second.x);
    it_roi_prediction->second.y = roi_current.y + (roi_current.y - it_roi_prev_request->second.y);
    it_roi_prediction->second.width = roi_current.width + (roi_current.width - it_roi_prev_request->second.width);
    it_roi_prediction->second.height = roi_current.height + (roi_current.height - it_roi_prev_request->second.height);
  }

  it_roi_prev_request->second = roi_current;
}

void TileCache::createDirectories(const std::string &toplevel, const std::vector<std::string> &layer_names, const std::string &tile_tree)
{
  for (const auto &layer_name : layer_names)
  {
    io::createDir(toplevel + layer_name + tile_tree);
  }
}