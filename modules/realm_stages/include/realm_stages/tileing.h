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

namespace realm
{
namespace stages
{

class Tileing : public StageBase
{
  public:
    using Ptr = std::shared_ptr<Tileing>;
    using ConstPtr = std::shared_ptr<const Tileing>;

    struct SaveSettings
    {

    };

  public:
    explicit Tileing(const StageSettings::Ptr &stage_set, double rate);
    ~Tileing();
    void addFrame(const Frame::Ptr &frame) override;
    bool process() override;
    void saveAll();

  private:
    std::deque<Frame::Ptr> m_buffer;
    std::mutex m_mutex_buffer;

    SaveSettings m_settings_save;

    UTMPose::Ptr m_utm_reference;

    /// Warper to transform incoming grid maps from UTM coordinates to Web Mercator (EPSG:3857)
    gis::GdalWarper m_warper;

    MapTiler::Ptr m_map_tiler;
    TileCache::Ptr m_tile_cache;

    Tile::Ptr merge(const Tile::Ptr &t1, const Tile::Ptr &t2);
    Tile::Ptr blend(const Tile::Ptr &t1, const Tile::Ptr &t2);

    void finishCallback() override;
    void printSettingsToLog() override;

    void reset() override;
    void initStageCallback() override;
    uint32_t getQueueDepth() override;

    void publish(const Frame::Ptr &frame, const CvGridMap::Ptr &global_map, const CvGridMap::Ptr &update, uint64_t timestamp);

    void saveIter(uint32_t id, const CvGridMap::Ptr &map_update);
    Frame::Ptr getNewFrame();
};

} // namespace stages
} // namespace realm

#endif //PROJECT_TILEING_H
