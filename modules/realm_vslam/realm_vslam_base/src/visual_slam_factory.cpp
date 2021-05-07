

#include <realm_vslam_base/visual_slam_factory.h>

#if defined USE_ORB_SLAM2 || defined USE_ORB_SLAM3
  #include <realm_vslam_base/orb_slam.h>
#endif

#ifdef USE_OPEN_VSLAM
  #include <realm_vslam_base/open_vslam.h>
#endif

#ifdef USE_SVO
  #include <realm_vslam_base/svo.h>
#endif

#ifdef USE_SVO2
  #include <realm_vslam_base/svo2.h>
#endif

#ifdef USE_DSO
  #include <realm_vslam_base/dso.h>
#endif

#ifdef USE_OV2SLAM
  #include <realm_vslam_base/ov2_slam.h>
#endif

using namespace realm;

VisualSlamIF::Ptr VisualSlamFactory::create(const VisualSlamSettings::Ptr &vslam_set,
                                            const CameraSettings::Ptr &cam_set,
                                            const ImuSettings::Ptr &imu_set)
{
  // If compiled with ORB SLAM 2
#ifdef USE_ORB_SLAM2
  if ((*vslam_set)["type"].toString() == "ORB_SLAM2")
      return std::make_shared<OrbSlam>(vslam_set, cam_set);
#endif

#ifdef USE_ORB_SLAM3
  if ((*vslam_set)["type"].toString() == "ORB_SLAM3")
  {
    if (imu_set != nullptr)
      return std::make_shared<OrbSlam>(vslam_set, cam_set, imu_set);
    else
      return std::make_shared<OrbSlam>(vslam_set, cam_set);
  }
#endif

#ifdef USE_OPEN_VSLAM
  if ((*vslam_set)["type"].toString() == "OPEN_VSLAM")
    return std::make_shared<OpenVslam>(vslam_set, cam_set);
#endif

#ifdef USE_SVO
  if ((*vslam_set)["type"].toString() == "SVO")
    return std::make_shared<SVO>(vslam_set, cam_set);
#endif

#ifdef USE_SVO2
  if ((*vslam_set)["type"].toString() == "SVO2")
    return std::make_shared<SVO2>(vslam_set, cam_set);
#endif

#ifdef USE_DSO
  if ((*vslam_set)["type"].toString() == "DSO")
    return std::make_shared<DSO>(vslam_set, cam_set);
#endif

#ifdef USE_OV2SLAM
  if ((*vslam_set)["type"].toString() == "OV2SLAM")
    return std::make_shared<Ov2Slam>(vslam_set, cam_set);
#endif
  throw std::invalid_argument("Error: SLAM framework '" + (*vslam_set)["type"].toString() + "' not found");
}