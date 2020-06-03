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
public:
  using Ptr = std::shared_ptr<VisualSlamIF>;
  using ConstPtr = std::shared_ptr<const VisualSlamIF>;
  using ResetFuncCb = std::function<void(void)>;
  using PoseUpdateFuncCb = std::function<void(int, const cv::Mat &, const cv::Mat &)>;
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

//=====================================================================================================================
//=======  Core
//=====================================================================================================================

  /*!
   * @brief Essential tracking function that takes the current frame, processes it inside the derived visual SLAM framework and
   * sets the pose. It returns the current state of the visual SLAM to let the caller know, if processing was successfull.
   * @param frame Input frame to be tracked.
   * @param T_c2w_initial There is the possibility to provide an initial guess of the frame pose to allow more robust
   * tracking. This can be considered optional, if a default argument (cv::Mat()) is provided.
   * @return State of the visual SLAM
   */
  virtual State track(Frame::Ptr &frame, const cv::Mat &T_c2w_initial) = 0;

  /*!
   * @brief Closing the visual SLAM will trigger a controlled shutdown of the derived framework. It is not expected
   * to be reopened after it was closed once. Therefore it can be considered the last step in cleaning up.
   */
  virtual void close() = 0;

  /*!
   * @brief Reset will be called in several circumstances. But in most cases the user demands a reset because the SLAM
   * got lost and is not expected to relocalize. Reset must clear all parameters of the visual SLAM framework, so it can
   * be assumed that not prior knowledge exists. There might be soft resets in the future to keep an existing map, but right
   * now we only support the full reset.
   */
  virtual void reset() = 0;

  /*!
   * @brief As the name suggests this function should print all parameters of the visual SLAM framework into the log
   * instance. It helps to retrace possible bugs while running the application and store it permanently for later evaluation.
   */
  virtual void printSettingsToLog() = 0;

//====================================================================================================================
//=======  Optional
//====================================================================================================================

  virtual cv::Mat getMapPoints() const
  {};

  virtual cv::Mat getTrackedMapPoints() const
  {};

  virtual bool drawTrackedImage(cv::Mat &img) const
  {};

  // Callbacks
  virtual void registerUpdateTransport(const PoseUpdateFuncCb &func)
  {};

  virtual void registerResetCallback(const ResetFuncCb &func)
  {};

private:
};

} // namespace realm

#endif //PROJECT_VISUAL_SLAM_IF_H