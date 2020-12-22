/**
* This file is part of OpenREALM.
*
* Copyright (C) 2020 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
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

#ifndef OPENREALM_CV_IMPORT_H
#define OPENREALM_CV_IMPORT_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace realm
{
namespace io
{

cv::Mat loadImage(const std::string &filepath);

cv::Mat loadImageFromBinary(const std::string &filepath);

} // namespace io
} // namespace realm

#endif //OPENREALM_CV_IMPORT_H
