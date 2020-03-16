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

#ifndef PROJECT_ANALYSIS_H
#define PROJECT_ANALYSIS_H

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

namespace realm
{
namespace analysis
{

/*!
 * @brief Converts a single channel floating point mat to a RGB color map
 * @param img CV_32FC1 or CV_64FC1 floating point mat
 * @param mask Mask of valid pixels
 * @param flag Color layout
 * @return Colormap of input mat
 */
cv::Mat convertToColorMapFromCVFC1(const cv::Mat &img, const cv::Mat &mask, cv::ColormapTypes flag);

/*!
 * @brief Converts a three channel floating point mat to a RGB color map
 * @param img CV_32FC3 or CV_64FC3 floating point mat
 * @param mask Mask of valid pixels
 * @return Colormap of input mat
 */
cv::Mat convertToColorMapFromCVFC3(const cv::Mat &img, const cv::Mat &mask);

} // namespace analysis
} // namespace realm

#endif //PROJECT_ANALYSIS_H
