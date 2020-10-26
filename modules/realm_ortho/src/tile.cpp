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

#include <tile.h>

using namespace realm;

Tile::Tile(int zoom_level, int tx, int ty, const cv::Mat &img)
 : _zoom_level(zoom_level),
   _index(tx, ty),
   _img(img)
{
}

void Tile::lock()
{
  _mutex_data.lock();
}

void Tile::unlock()
{
  _mutex_data.unlock();
}

int Tile::zoom_level() const
{
  return _zoom_level;
}

int Tile::x() const
{
  return _index.x;
}

int Tile::y() const
{
  return _index.y;
}

cv::Mat& Tile::data()
{
  return _img;
}