

#include <realm_ortho/tile.h>

using namespace realm;

Tile::Tile(int zoom_level, int tx, int ty, const CvGridMap &map, bool is_tms)
 : m_zoom_level(zoom_level),
   m_index(tx, ty),
   m_data(std::make_shared<CvGridMap>(map)),
   m_tms(is_tms)
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

bool Tile::is_tms() const
{
  return m_tms;
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