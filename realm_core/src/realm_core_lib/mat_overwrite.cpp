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

#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <realm_core/mat_overwrite.h>

void realm::addMat(const cv::Mat &from, const cv::Mat &to, cv::Mat &result, const int &flag)
{
  // Allow addition only of matrice of same type
  assert(from.type() == to.type());
  assert(from.rows == to.rows && from.cols == to.cols);

  // Execute addition by flag
  if (flag == REALM_OVERWRITE_ALL)
  {
    internal::overwriteAll(from, to, result);
    return;
  }
  if (flag == REALM_OVERWRITE_NONE)
  {
    internal::overwriteNone(from, to, result);
    return;
  }
  if (flag == REALM_OVERWRITE_ZERO)
  {
    switch(to.type())
    {
      case CV_32FC1: internal::overwriteNoValues<float>(from, to, result); break;
      case CV_32FC3: internal::overwriteNoValues<cv::Vec3f>(from, to, result); break;
      case CV_64F: internal::overwriteNoValues<double>(from, to, result); break;
      case CV_16UC1: internal::overwriteNoValues<uint16_t>(from, to, result); break;
      case CV_8UC1: internal::overwriteNoValues<uchar>(from, to, result); break;
      case CV_8UC3: internal::overwriteNoValues<cv::Vec3b>(from, to, result); break;
      case CV_8UC4: internal::overwriteNoValues<cv::Vec4b>(from, to, result); break;
      default: break;
    }
    return;
  }
  if (flag == REALM_OVERWRITE_WITH_NON_ZERO)
  {
    switch(to.type())
    {
      case CV_32FC1: internal::overwriteWithValues<float>(from, to, result); break;
      case CV_32FC3: internal::overwriteWithValues<cv::Vec3f>(from, to, result); break;
      case CV_64F: internal::overwriteWithValues<double>(from, to, result); break;
      case CV_16UC1: internal::overwriteWithValues<uint16_t>(from, to, result); break;
      case CV_8UC1: internal::overwriteWithValues<uchar>(from, to, result); break;
      case CV_8UC3: internal::overwriteWithValues<cv::Vec3b>(from, to, result); break;
      case CV_8UC4: internal::overwriteWithValues<cv::Vec4b>(from, to, result); break;
      default: break;
    }
    return;
  }
}

cv::Mat realm::internal::cvtToMono(const cv::Mat &src)
{
  cv::Mat dst;
  switch(src.type())
  {
    case CV_32FC1:
      dst = src; break;
    case CV_32FC3:
      throw(std::invalid_argument("Error: Conversion to mono for CV_32FC3 not defined."));
    case CV_64F:
      dst = src; break;
    case CV_16UC1:
      dst =  src; break;
    case CV_8UC1:
      dst =  src; break;
    case CV_8UC3:
      cv::cvtColor(src, dst, CV_BGR2GRAY); break;
    case CV_8UC4:
      cv::cvtColor(src, dst, CV_BGRA2GRAY); break;
    default:
      throw(std::invalid_argument("Input array of matrix addition has wrong type!"));
  }
  return dst;
}

void realm::internal::overwriteAll(const cv::Mat &from, const cv::Mat &to, cv::Mat &result)
{
  result = from;
}

void realm::internal::overwriteNone(const cv::Mat &from, const cv::Mat &to, cv::Mat &result)
{
  result = to;
}

template<typename T>
void realm::internal::overwriteNoValues(const cv::Mat &from, const cv::Mat &to, cv::Mat &result)
{
  assert(&from != &result);
  result = cv::Mat::zeros(to.rows, to.cols, to.type());
  for (uint32_t r = 0; r < to.rows; ++r)
    for (uint32_t c = 0; c < to.cols; ++c)
    {
      const auto &val_to = to.at<T>(r, c);
      if (!isNoValue(val_to))
      {
        result.at<T>(r, c) = val_to;
        continue;
      }
      const auto &val_from = from.at<T>(r, c);
      if (!isNoValue(val_from))
      {
        result.at<T>(r, c) = val_from;
        continue;
      }
    }
}

template<typename T>
void realm::internal::overwriteWithValues(const cv::Mat &from, const cv::Mat &to, cv::Mat &result)
{
  result = cv::Mat::zeros(to.rows, to.cols, to.type());
  for (uint32_t r = 0; r < to.rows; ++r)
    for (uint32_t c = 0; c < to.cols; ++c)
    {
      const auto &val_from = from.at<T>(r, c);
      if (!isNoValue(val_from))
      {
        result.at<T>(r, c) = val_from;
        continue;
      }
      const auto &val_to = to.at<T>(r, c);
      if (!isNoValue(val_to))
      {
        result.at<T>(r, c) = val_to;
        continue;
      }
    }
}