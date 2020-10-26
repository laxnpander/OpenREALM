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

namespace realm
{

class Tile
{
public:
  using Ptr = std::shared_ptr<Tile>;

public:
  Tile(int zoom_level, int tx, int ty, const cv::Mat &img);

  void lock();
  void unlock();

  int zoom_level() const;

  int x() const;
  int y() const;

  cv::Mat& data();

private:

  int _zoom_level;

  cv::Point2i _index;

  cv::Mat _img;

  std::mutex _mutex_data;
};

} // namespace realm

#endif //GENERAL_TESTBED_TILE_H
