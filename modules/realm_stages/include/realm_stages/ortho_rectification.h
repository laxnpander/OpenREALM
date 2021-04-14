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
        bool save_ortho_rgb;
        bool save_ortho_gtiff;
        bool save_elevation;
        bool save_elevation_angle;

      bool save_required()
      {
        return save_ortho_rgb || save_ortho_gtiff ||
               save_elevation || save_elevation_angle;
      }
    };

  public:
    explicit OrthoRectification(const StageSettings::Ptr &stage_set, double rate);
    void addFrame(const Frame::Ptr &frame) override;
    bool process() override;
  private:

    bool m_do_publish_pointcloud;

    double m_GSD;
    SaveSettings m_settings_save;

    std::deque<Frame::Ptr> m_buffer;
    std::mutex m_mutex_buffer;

    void reset() override;
    void initStageCallback() override;
    void printSettingsToLog() override;
    uint32_t getQueueDepth() override;

    void saveIter(const CvGridMap& surface_model, const CvGridMap& orthophoto, uint8_t zone, char band, uint32_t id);
    void publish(const Frame::Ptr &frame);
    Frame::Ptr getNewFrame();
};

} // namespace stages
} // namespace realm

#endif //PROJECT_ORTHO_RECTIFICATION_H
