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

#include <realm_ortho/tile.h>

using namespace realm;

Tile::Tile(int zoom_level, int tx, int ty, const CvGridMap &map)
 : m_zoom_level(zoom_level),
   m_index(tx, ty),
   m_data(std::make_shared<CvGridMap>(map))
{
}

void Tile::lock()
{
  m_mutex_data.lock();
}

void Tile::unlock()
{
  m_mutex_data.unlock();
}

int Tile::zoom_level() const
{
  return m_zoom_level;
}

int Tile::x() const
{
  return m_index.x;
}

int Tile::y() const
{
  return m_index.y;
}

CvGridMap::Ptr& Tile::data()
{
  return m_data;
}