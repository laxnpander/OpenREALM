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

#include <realm_vslam/visual_slam_factory.h>

using namespace realm;

VisualSlamIF::Ptr VisualSlamFactory::create(const VisualSlamSettings::Ptr &vslam_set,
                                            const CameraSettings::Ptr &cam_set)
{
  // If compiled with ORB SLAM 2
#ifdef USE_ORB_SLAM2
  if (vslam_set->get<std::string>("type") == "ORB_SLAM2")
    return std::make_shared<OrbSlam2>(vslam_set, cam_set);
#elif USE_SVO
  if (vslam_set->get<std::string>("type") == "SVO")
    return std::make_shared<SVO>(vslam_set, cam_set);
#elif USE_SVO2
  if (vslam_set->get<std::string>("type") == "SVO2")
    return std::make_shared<SVO2>(vslam_set, cam_set);
#elif USE_DSO
  if (vslam_set->get<std::string>("type") == "DSO")
    return std::make_shared<DSO>(vslam_set, cam_set);
#endif
  throw std::invalid_argument("Error: SLAM framework '" + vslam_set->get<std::string>("type") + "' not found");
}