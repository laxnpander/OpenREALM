

#ifndef PROJECT_VISUAL_SLAM_FACTORY_H
#define PROJECT_VISUAL_SLAM_FACTORY_H

#include <realm_core/camera.h>
#include <realm_core/camera_settings.h>
#include <realm_core/imu_settings.h>
#include <realm_vslam_base/visual_slam_IF.h>
#include <realm_vslam_base/visual_slam_settings.h>

namespace realm
{

class VisualSlamFactory
{
  public:
    static VisualSlamIF::Ptr create(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set, const ImuSettings::Ptr &imu_set = nullptr);
};

}

#endif //PROJECT_VISUAL_SLAM_FACTORY_H
