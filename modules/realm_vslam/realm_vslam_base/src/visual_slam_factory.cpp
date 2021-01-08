

#include <realm_vslam_base/visual_slam_factory.h>

using namespace realm;

VisualSlamIF::Ptr VisualSlamFactory::create(const VisualSlamSettings::Ptr &vslam_set,
                                            const CameraSettings::Ptr &cam_set)
{
  // If compiled with ORB SLAM 2
#ifdef USE_ORB_SLAM3
  if ((*vslam_set)["type"].toString() == "ORB_SLAM3")
    return std::make_shared<OrbSlam>(vslam_set, cam_set);
#elif USE_OPEN_VSLAM
  if ((*vslam_set)["type"].toString() == "OPEN_VSLAM")
    return std::make_shared<OpenVslam>(vslam_set, cam_set);
#elif USE_SVO
  if ((*vslam_set)["type"].toString() == "SVO")
    return std::make_shared<SVO>(vslam_set, cam_set);
#elif USE_SVO2
  if ((*vslam_set)["type"].toString() == "SVO2")
    return std::make_shared<SVO2>(vslam_set, cam_set);
#elif USE_DSO
  if ((*vslam_set)["type"].toString() == "DSO")
    return std::make_shared<DSO>(vslam_set, cam_set);
#endif
  throw std::invalid_argument("Error: SLAM framework '" + (*vslam_set)["type"].toString() + "' not found");
}