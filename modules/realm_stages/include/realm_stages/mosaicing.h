

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
#include <realm_io/gis_export.h>
#include <realm_io/gdal_continuous_writer.h>
#include <realm_io/utilities.h>
//#include <realm_ortho/delaunay_2d.h>

#ifdef WITH_PCL
#include <realm_io/pcl_export.h>
#endif

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
        bool split_gtiff_channels;
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

  public:
    explicit Mosaicing(const StageSettings::Ptr &stage_set, double rate);
    ~Mosaicing();
    void addFrame(const Frame::Ptr &frame) override;
    bool process() override;
    void runPostProcessing();
    void saveAll();

  private:
    std::deque<Frame::Ptr> m_buffer;
    std::mutex m_mutex_buffer;

    //! Publish of mesh is optional. Set >0 if should be published. Additionally it can be downsampled.
    int m_publish_mesh_nth_iter;
    int m_publish_mesh_every_nth_kf;
    bool m_do_publish_mesh_at_finish;
    double m_downsample_publish_mesh; // [m/pix]

    bool m_use_surface_normals;

    int m_th_elevation_min_nobs;
    float m_th_elevation_var;

    SaveSettings m_settings_save;

    UTMPose::Ptr m_utm_reference;
    CvGridMap::Ptr m_global_map;
    //Delaunay2D::Ptr m_mesher;
    io::GDALContinuousWriter::Ptr m_gdal_writer;

    void finishCallback() override;
    void printSettingsToLog() override;
    uint32_t getQueueDepth() override;

    CvGridMap blend(CvGridMap::Overlap *overlap);

    void reset() override;
    void initStageCallback() override;
    std::vector<Face> createMeshFaces(const CvGridMap::Ptr &map);

    void publish(const Frame::Ptr &frame, const CvGridMap::Ptr &global_map, const CvGridMap::Ptr &update, uint64_t timestamp);

    void saveIter(uint32_t id, const CvGridMap::Ptr &map_update);
    Frame::Ptr getNewFrame();
};

} // namespace stages
} // namespace realm

#endif //PROJECT_MOSAICING_H
