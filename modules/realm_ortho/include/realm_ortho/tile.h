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
 * Tiled Map Service specification and a multi-layered grid map holding the data.
 */
class Tile
{
public:
  using Ptr = std::shared_ptr<Tile>;

public:
  /*!
   * @brief Non-default constructor
   * @param zoom_level Zoom level or z-coordinate according to TMS standard
   * @param tx Tile index in x-direction according to TMS standard
   * @param ty Tile index in y-direction according to TMS standard
   * @param map Multi-layered grid map holding the data for the tile
   */
  Tile(int zoom_level, int tx, int ty, const CvGridMap &map);

  /*!
   * @brief Locks the tile when being accessed or modified to prevent multi-threading problems.
   */
  void lock();

  /*!
   * @brief Releases the lock on the tile for other processes to access or write it.
   */
  void unlock();

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

  /// Zoom level according to TMS standard
  int _zoom_level;

  /// Tile index according to TMS standard
  cv::Point2i _index;

  /// Multi-layered grid map container
  CvGridMap::Ptr _data;

  /// Main mutex to prevent simultaneous access from different threads
  std::mutex _mutex_data;
};

} // namespace realm

#endif //GENERAL_TESTBED_TILE_H
