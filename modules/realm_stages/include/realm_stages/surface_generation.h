

#ifndef PROJECT_SURFACE_GENERATION_H
#define PROJECT_SURFACE_GENERATION_H

#include <iostream>
#include <deque>

#include <realm_stages/stage_base.h>
#include <realm_stages/stage_settings.h>
#include <realm_core/frame.h>
#include <realm_core/stereo.h>
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
        bool save_elevation;
        bool save_normals;

      bool save_required()
      {
        return save_elevation || save_normals;
      }
    };

  public:
    explicit SurfaceGeneration(const StageSettings::Ptr &settings, double rate);
    void addFrame(const Frame::Ptr &frame) override;
    bool process() override;
    bool changeParam(const std::string& name, const std::string &val) override;
  private:

    std::mutex m_mutex_params;
    bool m_try_use_elevation;
    bool m_compute_all_frames;

    int m_knn_max_iter;

    bool m_is_projection_plane_offset_computed;
    double m_projection_plane_offset;

    ortho::DigitalSurfaceModel::SurfaceNormalMode m_mode_surface_normals;

    SaveSettings m_settings_save;

    Plane m_plane_reference;

    std::deque<Frame::Ptr> m_buffer;
    std::mutex m_mutex_buffer;

    void reset() override;
    void initStageCallback() override;
    void printSettingsToLog() override;
    uint32_t getQueueDepth() override;

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
    ortho::DigitalSurfaceModel::Ptr createPlanarSurface(const Frame::Ptr &frame);
    ortho::DigitalSurfaceModel::Ptr createElevationSurface(const Frame::Ptr &frame);
};

} // namespace stages
} // namespace realm

#endif //PROJECT_SURFACE_GENERATION_H
