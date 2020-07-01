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

#ifndef PROJECT_ORTHO_RECTIFICATION_H
#define PROJECT_ORTHO_RECTIFICATION_H

#include <deque>
#include <chrono>

#include <realm_io/cv_export.h>
#include <realm_io/gis_export.h>
#include <realm_io/utilities.h>
#include <realm_stages/stage_base.h>
#include <realm_stages/conversions.h>
#include <realm_stages/stage_settings.h>
#include <realm_core/frame.h>
#include <realm_ortho/rectification.h>

namespace realm
{
namespace stages
{

class OrthoRectification : public StageBase
{
  public:
    using Ptr = std::shared_ptr<OrthoRectification>;
    using ConstPtr = std::shared_ptr<const OrthoRectification>;

    struct SaveSettings
    {
        bool save_valid;
        bool save_ortho_rgb;
        bool save_ortho_gtiff;
        bool save_elevation;
        bool save_elevation_angle;
    };

  public:
    explicit OrthoRectification(const StageSettings::Ptr &stage_set, double rate);
    void addFrame(const Frame::Ptr &frame) override;
    bool process() override;
  private:

    double _GSD;
    SaveSettings _settings_save;

    std::deque<Frame::Ptr> _buffer;
    std::mutex _mutex_buffer;

    void reset() override;
    void initStageCallback() override;
    void printSettingsToLog() override;

    void saveIter(const CvGridMap& map, uint8_t zone, uint32_t id);
    void publish(const Frame::Ptr &frame);
    Frame::Ptr getNewFrame();
};

} // namespace stages
} // namespace realm

#endif //PROJECT_ORTHO_RECTIFICATION_H
