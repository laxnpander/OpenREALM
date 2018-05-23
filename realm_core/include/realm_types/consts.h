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

#ifndef PROJECT_REALM_TYPE_CONSTS_H
#define PROJECT_REALM_TYPE_CONSTS_H

#include <opencv2/core.hpp>

namespace realm
{
namespace consts
{

const float FLOAT_NO_VALUE = -32767;
const float FLOAT_EPSILON = 0.0001;

const double DOUBLE_NO_VALUE = -32767;
const double DOUBLE_EPSILON = 0.0001;

const int INT_NO_VALUE = 0;
const int INT_EPSILON = 1;

inline float getNoValue(float &id) { return FLOAT_NO_VALUE; }

inline double getNoValue(double &id) { return DOUBLE_NO_VALUE; }

inline uchar getNoValue(uchar id) { return (uchar)INT_NO_VALUE; }

inline uint16_t getNoValue(uint16_t) { return (uint16_t)INT_NO_VALUE; }

inline cv::Vec3b getNoValue(const cv::Vec3b &id) { return cv::Vec3b(INT_NO_VALUE, INT_NO_VALUE, INT_NO_VALUE); }

inline cv::Vec3f getNoValue(const cv::Vec3f &id) { return cv::Vec3f(FLOAT_NO_VALUE, FLOAT_NO_VALUE, FLOAT_NO_VALUE); }

inline cv::Vec4b getNoValue(const cv::Vec4b &id) { return cv::Vec4b(INT_NO_VALUE, INT_NO_VALUE, INT_NO_VALUE, INT_NO_VALUE); }

template<typename T>
T getNoValue()
{
  T id = 0;
  return consts::getNoValue(id);
}

} // namespace consts
} // namespace realm

#endif //PROJECT_CONSTS_H
