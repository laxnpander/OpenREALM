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

#ifndef PROJECT_MOSAICING_H
#define PROJECT_MOSAICING_H

#include <deque>
#include <chrono>

#include <realm_stages/stage_base.h>
#include <realm_stages/conversions.h>
#include <realm_stages/stage_settings.h>
#include <realm_core/frame.h>
#include <realm_core/cv_grid_map.h>
#include <realm_core/analysis.h>
#include <realm_io/cv_export.h>
#include <realm_io/pcl_export.h>
#include "realm_io/gis_export.h"
#include <realm_io/utilities.h>
#include <realm_ortho/delaunay_2d.h>

namespace realm
{
namespace stages
{

class Mosaicing : public StageBase
{
  public:
    using Ptr = std::shared_ptr<Mosaicing>;
    using ConstPtr = std::shared_ptr<const Mosaicing>;

    struct SaveSettings
    {
        bool save_valid;
        bool save_ortho_rgb_one;
        bool save_ortho_rgb_all;
        bool save_ortho_gtiff_one;
        bool save_ortho_gtiff_all;
        bool save_elevation_one;
        bool save_elevation_all;
        bool save_elevation_var_one;
        bool save_elevation_var_all;
        bool save_elevation_obs_angle_one;
        bool save_elevation_obs_angle_all;
        bool save_elevation_mesh_one;
        bool save_num_obs_one;
        bool save_num_obs_all;
        bool save_dense_ply;
    };

    struct GridQuickAccess
    {
      public:
        using Ptr = std::shared_ptr<GridQuickAccess>;
        using ConstPtr = std::shared_ptr<const GridQuickAccess>;
      public:
        GridQuickAccess(const std::vector<std::string> &layer_names, const CvGridMap &map);
        void move(int row, int col);

        float* ele;         // elevation at row, col
        float* var;         // elevation variance at row, col
        float* hyp;         // elevation hypothesis at row, col
        float* angle;       // elevation observation angle at row, col
        uint16_t* nobs;     // number of observations at row, col
        cv::Vec3f* normal;  // surface normal
        cv::Vec4b* rgb;     // color at row, col
        uchar* elevated;    // elevation computed at row, col
        uchar* valid;       // valid at row, col
      private:
        cv::Mat _elevation;
        cv::Mat _elevation_normal;
        cv::Mat _elevation_var;
        cv::Mat _elevation_hyp;
        cv::Mat _elevation_angle;
        cv::Mat _elevated;
        cv::Mat _num_observations;
        cv::Mat _color_rgb;
        cv::Mat _valid;
    };

  public:
    explicit Mosaicing(const StageSettings::Ptr &stage_set);
    void addFrame(const Frame::Ptr &frame) override;
    bool process() override;
    void runPostProcessing();
    void saveAll();
  private:
    std::deque<Frame::Ptr> _buffer;
    std::mutex _mutex_buffer;

    //! Publish of mesh is optional. Set >0 if should be published. Additionally it can be downsampled.
    int _publish_mesh_nth_iter;
    int _publish_mesh_every_nth_kf;
    bool _do_publish_mesh_at_finish;
    double _downsample_publish_mesh; // [m/pix]

    bool _use_surface_normals;

    int _th_elevation_min_nobs;
    float _th_elevation_var;

    SaveSettings _settings_save;

    UTMPose::Ptr _utm_reference;
    CvGridMap::Ptr _global_map;
    Delaunay2D::Ptr _mesher;

    void finishCallback() override;
    void printSettingsToLog() override;

    CvGridMap blend(CvGridMap::Overlap *overlap);

    void setGridElement(const GridQuickAccess::Ptr &ref, const GridQuickAccess::Ptr &inp);
    void updateGridElement(const GridQuickAccess::Ptr &ref, const GridQuickAccess::Ptr &inp);

    void reset() override;
    void initStageCallback() override;
    std::vector<Face> createMeshFaces(const CvGridMap::Ptr &map);

    void publish(const Frame::Ptr &frame, const CvGridMap::Ptr &global_map, const CvGridMap::Ptr &update, uint64_t timestamp);

    void saveIter(uint32_t id);
    Frame::Ptr getNewFrame();
};

} // namespace stages
} // namespace realm

#endif //PROJECT_MOSAICING_H
