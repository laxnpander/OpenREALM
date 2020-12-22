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

#include <realm_vslam_base/dummy_referencer.h>

#include <realm_core/loguru.h>

using namespace realm;

DummyReferencer::DummyReferencer(const cv::Mat &T_c2g)
  : m_transformation_c2g(T_c2g)
{
}

void DummyReferencer::init(const std::vector<Frame::Ptr> &frames)
{
  LOG_F(WARNING, "This is a dummy georeferenciation. You have to manually provide the transformation from camera to world frame. "
                 "Not call to 'init()' possible!");
}

cv::Mat DummyReferencer::getTransformation()
{
  std::unique_lock<std::mutex> lock(m_mutex_t_c2g);
  return m_transformation_c2g;
}

void DummyReferencer::update(const Frame::Ptr &frame)
{
}

bool DummyReferencer::isBuisy()
{
  return false;
}

bool DummyReferencer::isInitialized()
{
  return true;
}