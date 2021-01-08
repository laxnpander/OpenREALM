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

#ifndef PROJECT_CV_EXPORT_H
#define PROJECT_CV_EXPORT_H

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <realm_core/frame.h>
#include <realm_core/analysis.h>
#include <realm_io/utilities.h>

namespace realm
{
namespace io
{

enum class ColormapType
{
    DEPTH,
    ELEVATION,
    NORMALS,
    NUM_OBS
};

void saveStereoPair(const Frame::Ptr &frame_left,
                    const Frame::Ptr &frame_right,
                    const std::string &path);

void saveImage(const cv::Mat &img,
               const std::string &name);

void saveImageToBinary(const cv::Mat &data,
                       const std::string &filepath);

void saveDepthMap(const cv::Mat &img,
                  const std::string &filename,
                  uint32_t id);

void saveImageColorMap(const cv::Mat &img,
                       float range_min,
                       float range_max,
                       const std::string &directory,
                       const std::string &name,
                       uint32_t frame_id,
                       ColormapType flag);

void saveImageColorMap(const cv::Mat &img,
                       const cv::Mat &mask,
                       const std::string &directory,
                       const std::string &name,
                       ColormapType flag);

void saveImageColorMap(const cv::Mat &img,
                       const cv::Mat &mask,
                       const std::string &directory,
                       const std::string &name,
                       uint32_t frame_id,
                       ColormapType flag);

void saveImageColorMap(const cv::Mat &img,
                       const cv::Mat &mask,
                       const std::string &name,
                       ColormapType flag);

void saveCvGridMapLayer(const CvGridMap &map,
                        int zone,
                        char band,
                        const std::string &layer_name,
                        const std::string &filename);

void saveCvGridMapMeta(const CvGridMap &map,
                       int zone,
                       char band,
                       const std::string &filename);

} // namespace io
} // namespace realm

#endif //PROJECT_CV_EXPORT_H
