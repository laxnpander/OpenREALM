

#ifndef GENERAL_TESTBED_TILE_H
#define GENERAL_TESTBED_TILE_H

#include <mutex>
#include <memory>
#include <opencv2/core.hpp>

#include <realm_core/cv_grid_map.h>

namespace realm
{

/*!
 * @brief Tile is a container class that is defined by a coordinate (x,y) in a specific zoom level following the
 * Tiled Map Service or Google/OSM specification and a multi-layered grid map holding the data.
 */
class Tile
{
public:
  using Ptr = std::shared_ptr<Tile>;

public:
  /*!
   * @brief Non-default constructor
   * @param zoom_level Zoom level or z-coordinate according to TMS or Google/OSM specification
   * @param tx Tile index in x-direction according to TMS or Google/OSM specification
   * @param ty Tile index in y-direction according to TMS or Google/OSM specification
   * @param map Multi-layered grid map holding the data for the tile
   * @param is_tms Indicates this is a TMS rather than google tile
   */
  Tile(int zoom_level, int tx, int ty, const CvGridMap &map, bool is_tms);

  /*!
   * @brief Locks the tile when being accessed or modified to prevent multi-threading problems.
   */
  void lock();

  /*!
   * @brief Releases the lock on the tile for other processes to access or write it.
   */
  void unlock();

  /*!
   * @brief Indicates if this is a TMS or Google/OSM tile
   * @return True if tms
   */
  bool is_tms() const;

  /*!
   * @brief Getter for the zoom level
   * @return Zoom level of the data
   */
  int zoom_level() const;

  /*!
   * @brief Getter for the tile index in x-direction
   * @return Tile index in x-direction (W/E)
   */
  int x() const;

  /*!
   * @brief Getter for the tile index in y-direction
   * @return Tile index in y-direction (N/S)
   */
  int y() const;

  /*!
   * @brief Getter for the multi-layered data container. Warning: Returned by reference, modifications will therefore
   * be permanent!
   * @return Multi-layered grid map containing the data for the tile, e.g. observed color, elevation and angle for
   * the specific tile are all saved inside the grid map.
   */
  CvGridMap::Ptr& data();

private:

  /// Zoom level according to TMS or Google/OSM specification
  int m_zoom_level;

  /// Tile index according to TMS or Google/OSM specification
  cv::Point2i m_index;

  /// Multi-layered grid map container
  CvGridMap::Ptr m_data;

  /// Indicates this is a tms tiles
  bool m_tms;

  /// Main mutex to prevent simultaneous access from different threads
  std::mutex m_mutex_data;
};

} // namespace realm

#endif //GENERAL_TESTBED_TILE_H
