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

#ifndef OPENREALM_DUMMY_REFERENCER_H
#define OPENREALM_DUMMY_REFERENCER_H

#include <realm_vslam_base/geospatial_referencer_IF.h>

namespace realm
{

class DummyReferencer : public GeospatialReferencerIF
{
public:
  explicit DummyReferencer(const cv::Mat &T_c2g);

  void init(const std::vector<Frame::Ptr> &frames) override;
  cv::Mat getTransformation() override;
  void update(const Frame::Ptr &frame) override;
  bool isBuisy() override;
  bool isInitialized() override;

private:

  std::mutex _mutex_t_c2g;
  cv::Mat _transformation_c2g;
};

} // namespace realm

#endif //OPENREALM_DUMMY_REFERENCER_H
