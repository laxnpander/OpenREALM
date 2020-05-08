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

#ifndef PROJECT_SURFACE_GENERATION_H
#define PROJECT_SURFACE_GENERATION_H

#include <iostream>
#include <deque>

#include <realm_stages/stage_base.h>
#include <realm_stages/stage_settings.h>
#include <realm_core/frame.h>
#include <realm_io/cv_export.h>
#include <realm_ortho/dsm.h>

namespace realm
{
namespace stages
{

class SurfaceGeneration : public StageBase
{
  public:
    struct SaveSettings
    {
        bool save_valid;
        bool save_elevation;
        bool save_normals;
    };

  public:
    explicit SurfaceGeneration(const StageSettings::Ptr &settings);
    void addFrame(const Frame::Ptr &frame) override;
    bool process() override;
    bool changeParam(const std::string& name, const std::string &val) override;
  private:

    std::mutex _mutex_params;
    bool _try_use_elevation;
    double _knn_radius_factor;

    bool _is_projection_plane_offset_computed;
    double _projection_plane_offset;

    DigitalSurfaceModel::SurfaceNormalMode _mode_surface_normals;

    SaveSettings _settings_save;

    Plane _plane_reference;

    std::deque<Frame::Ptr> _buffer;
    std::mutex _mutex_buffer;

    void reset() override;
    void initStageCallback() override;
    void printSettingsToLog() override;

    void saveIter(const CvGridMap &surface, uint32_t id);
    void publish(const Frame::Ptr &frame);

    Frame::Ptr getNewFrame();

    /*!
     * @brief Computes the offset of the projection plane by analyzing the sparse cloud of a frame.
     * @param frame Frame for which the plane offset should be computed
     * @return Offset of the projection plane from the x-y-plane. No rotations allowed
     */
    double computeProjectionPlaneOffset(const Frame::Ptr &frame);

    SurfaceAssumption computeSurfaceAssumption(const Frame::Ptr &frame);
    DigitalSurfaceModel::Ptr createPlanarSurface(const Frame::Ptr &frame);
    DigitalSurfaceModel::Ptr createElevationSurface(const Frame::Ptr &frame);
};

} // namespace stages
} // namespace realm

#endif //PROJECT_SURFACE_GENERATION_H
