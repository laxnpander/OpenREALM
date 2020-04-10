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

#ifndef PROJECT_MAT_OVERRIDE_H
#define PROJECT_MAT_OVERRIDE_H

#include <opencv2/core.hpp>

#include <realm_core/consts.h>
#include <realm_core/enums.h>

namespace realm
{

// TODO: find namespace name

/*!
 * @brief Several strategies implemented for matrix addition. Heavily used in CvGridMap for adding overlapping images
 * and handling data selection per cell in the overlapped region. Direction of addition is defined as "from", the
 * matrix data that should be added, in direction of "to", the existing matrix. It is easier to understand with the
 * following pictures:
 * - "to" equals a big global map, "from" equals a small amount of additional data for the big map
 * - "to" equals a sticker, "from" equals a table that the sticker is put onto
 * For template datatype conversion the following types were chosen:
 * - CV_32FC1 -> float
 * - CV_32FC3 -> cv::Vec3f
 * - CV_64F -> double
 * - CV_8UC1 -> uchar
 * - CV_8UC3 -> cv::Vec3b
 * - CV_8UC4 -> cv::Vec4b
 * @param from Matrix that should be added
 * @param to Matrix that is the destination of the addition
 * @param result result of the addition, might often be param "to"
 * @param flag addition flag, see enums or documentation below for further explanation
 */
void addMat(const cv::Mat &from, const cv::Mat &to, cv::Mat &result, const int &flag);

namespace internal
{

cv::Mat cvtToMono(const cv::Mat &src);

/*!
 * @brief Adding two matrice by overwriting all data of the destination matrix
 * @param from source matrix
 * @param to destination matrix
 * @param result output matrix, equals "from" in this case
 */
void overwriteAll(const cv::Mat &from, const cv::Mat &to, cv::Mat &result);

/*!
 * @brief Adding two matrice by overwriting no existing data
 * @param from source matrix
 * @param to destination matrix
 * @param result output matrix, equals "to" in this case
 */
void overwriteNone(const cv::Mat &from, const cv::Mat &to, cv::Mat &result);

/*!
 * @brief Adding two matrice by overwriting only data that is zero.
 * Note: See also the difference to only overwriting WITH elements that are not zero
 * @tparam T datatype conversion for accessing matrix elements, see also addMat
 * @param from source matrix
 * @param to destination matrix
 * @param result output matrix
 */
template<typename T>
void overwriteNoValues(const cv::Mat &from, const cv::Mat &to, cv::Mat &result);

/*!
 * @brief Adding two matrice by overwriting only with data that is not zero.
 * Note: See also the difference to only overwriting elements that are not zero
 * @tparam T datatype conversion for accessing matrix elements, see also addMat
 * @param from source matrix
 * @param to destination matrix
 * @param result output matrix
 */
template<typename T>
void overwriteWithValues(const cv::Mat &from, const cv::Mat &to, cv::Mat &result);

/*!
 * @brief Overloaded zero comparison for float (CV_32FC1)
 * @param val float value
 * @return true if below threshold
 */
inline bool isNoValue(float val) { return (val <= consts::FLOAT_NO_VALUE+consts::FLOAT_EPSILON); }

/*!
 * @brief Overloaded zero comparison for float (CV_32FC3)
 * @param val float vector of 3
 * @return true if below threshold
 */
inline bool isNoValue(const cv::Vec3f &val)
{
  return (val[0] <= consts::FLOAT_NO_VALUE+consts::FLOAT_EPSILON &&
          val[1] <= consts::FLOAT_NO_VALUE+consts::FLOAT_EPSILON &&
          val[2] <= consts::FLOAT_NO_VALUE+consts::FLOAT_EPSILON);
}

/*!
 * @brief Overloaded zero comparison for double (CV_64F)
 * @param val double value
 * @return true if below threshold
 */
inline bool isNoValue(double val) { return (fabs(val) <= consts::DOUBLE_NO_VALUE+consts::DOUBLE_EPSILON); }

/*!
 * @brief Overloaded zero comparison for uint16_t (CV_16UC1)
 * @param val uint16_t value
 * @return true if equals zero
 */
inline bool isNoValue(uint16_t val) { return (val == (uint16_t)consts::INT_NO_VALUE); }

/*!
 * @brief Overloaded zero comparison for uchar (CV_8UC1)
 * @param val uchar value
 * @return true if equals zero
 */
inline bool isNoValue(uchar val) { return (val == (uchar)consts::INT_NO_VALUE); }

/*!
 * @brief Overloaded zero comparison for Vec3b (CV_8UC3)
 * @param val vec3b value (BGR)
 * @return true of sum of all channels is zero
 */
inline bool isNoValue(cv::Vec3b val)
{
  return (val[0] == (uchar)consts::INT_NO_VALUE &&
          val[1] == (uchar)consts::INT_NO_VALUE &&
          val[2] == (uchar)consts::INT_NO_VALUE);
}

/*!
 * @brief Overloaded zero comparison for Vec4b (CV_8UC4)
 * @param val vec4b value (BGRA)
 * @return true of sum of all color channels is zero, alpha not included
 */
inline bool isNoValue(cv::Vec4b val)
{
  return (val[0] == (uchar)consts::INT_NO_VALUE &&
          val[1] == (uchar)consts::INT_NO_VALUE &&
          val[2] == (uchar)consts::INT_NO_VALUE &&
          val[3] == (uchar)consts::INT_NO_VALUE);
}

} // namespace internal
} // namespace realm

#endif //PROJECT_MAT_OVERRIDE_H
