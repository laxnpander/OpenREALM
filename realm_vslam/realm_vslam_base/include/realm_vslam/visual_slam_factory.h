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

#ifndef PROJECT_VISUAL_SLAM_FACTORY_H
#define PROJECT_VISUAL_SLAM_FACTORY_H

#include <realm_types/camera.h>
#include <realm_types/camera_settings.h>
#include <realm_vslam/visual_slam_IF.h>
#include <realm_vslam/visual_slam_settings.h>
#ifdef USE_ORB_SLAM2
  #include <realm_vslam/orb_slam.h>
#elif USE_SVO
  #include <realm_vslam/svo.h>
#elif USE_SVO2
  #include <realm_vslam/svo2.h>
#elif USE_DSO
  #include <realm_vslam/dso.h>
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
