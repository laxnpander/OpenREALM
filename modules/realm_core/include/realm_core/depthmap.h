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

#ifndef OPENREALM_DEPTHMAP_H
#define OPENREALM_DEPTHMAP_H

#include <memory>

#include <opencv2/core.hpp>

namespace realm
{

class Depthmap
{
public:
  using Ptr = std::shared_ptr<Depthmap>;

public:
  explicit Depthmap(const cv::Mat &data);

  double getMinDepth() const;

  double getMaxDepth() const;

  double getMedianDepth() const;

  cv::Mat data() const;

private:

  double _min_depth;
  double _med_depth;
  double _max_depth;

  cv::Mat _data;

  void updateDepthParameters();
};

}

#endif //OPENREALM_DEPTHMAP_H
