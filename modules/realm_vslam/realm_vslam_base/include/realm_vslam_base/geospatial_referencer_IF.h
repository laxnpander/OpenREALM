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

#ifndef PROJECT_GEOSPATIAL_REFERENCER_IF_H
#define PROJECT_GEOSPATIAL_REFERENCER_IF_H

#include <vector>
#include <memory>
#include <mutex>

#include <realm_core/frame.h>

namespace realm
{

class GeospatialReferencerIF
{
  public:
    using Ptr = std::shared_ptr<GeospatialReferencerIF>;
    using ConstPtr = std::shared_ptr<const GeospatialReferencerIF>;

  public:
    virtual void init(const std::vector<Frame::Ptr> &frames) = 0;
    virtual cv::Mat getTransformation() = 0;
    virtual void update(const Frame::Ptr &frame) = 0;
    virtual bool isBuisy() = 0;
    virtual bool isInitialized() = 0;
};

} // namespace realm

#endif //PROJECT_GEOSPATIAL_REFERENCER_IF_H
