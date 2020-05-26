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

#ifndef PROJECT_VISUAL_SLAM_IF_H
#define PROJECT_VISUAL_SLAM_IF_H

#include <memory>
#include <functional>
#include "opencv2/core/core.hpp"

#include <realm_core/loguru.h>
#include <realm_core/frame.h>

namespace realm
{

class VisualSlamIF
{
  // Typedefinition
  public:
    using Ptr = std::shared_ptr<VisualSlamIF>;
    using ConstPtr = std::shared_ptr<const VisualSlamIF>;
    using ResetFuncCb = std::function<void(void)>;
    using PoseUpdateFuncCb = std::function<void(int, const cv::Mat&, const cv::Mat &)>;
  public:
    enum class State
    {
        INITIALIZED,
        LOST,
        KEYFRAME_INSERT,
        FRAME_INSERT
    };
  // Class definition
  public:
    // Process Functions
    virtual State Track(Frame::Ptr &frame) = 0;
    virtual void Close() = 0;
    virtual void Reset() = 0;

    // Virtual Getter
    virtual cv::Mat GetMapPoints() const = 0;
    virtual cv::Mat GetTrackedMapPoints() const = 0;
    virtual bool DrawTrackedImage(cv::Mat &img) const = 0;

    // Callbacks
    virtual void RegisterUpdateTransport(const PoseUpdateFuncCb &func) = 0;
    virtual void RegisterResetCallback(const ResetFuncCb &func) = 0;

    virtual void printSettingsToLog() = 0;
  private:
};

} // namespace realm

#endif //PROJECT_VISUAL_SLAM_IF_H