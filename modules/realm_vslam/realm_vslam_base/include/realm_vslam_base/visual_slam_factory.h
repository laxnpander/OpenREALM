

#ifndef PROJECT_VISUAL_SLAM_FACTORY_H
#define PROJECT_VISUAL_SLAM_FACTORY_H

#include <realm_core/camera.h>
#include <realm_core/camera_settings.h>
#include <realm_vslam_base/visual_slam_IF.h>
#include <realm_vslam_base/visual_slam_settings.h>
#ifdef USE_ORB_SLAM3
  #include <realm_vslam_base/orb_slam.h>
#elif USE_OPEN_VSLAM
  #include <realm_vslam_base/open_vslam.h>
#elif USE_SVO
  #include <realm_vslam_base/svo.h>
#elif USE_SVO2
  #include <realm_vslam_base/svo2.h>
#elif USE_DSO
  #include <realm_vslam_base/dso.h>
#endif

namespace realm
{

class VisualSlamFactory
{
  public:
    static VisualSlamIF::Ptr create(const VisualSlamSettings::Ptr &vslam_set, const CameraSettings::Ptr &cam_set);
};

}

#endif //PROJECT_VISUAL_SLAM_FACTORY_H
