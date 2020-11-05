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

#include <realm_core/camera.h>

namespace realm
{

/*!
 * @brief Minimal depth container with basic functionalities to compute the minimum, maximum and median depth. Also
 * holds the camera model which was used for reconstruction, so reprojecting the depth into a point cloud is straightforward.
 */
class Depthmap
{
public:
  using Ptr = std::shared_ptr<Depthmap>;

public:
  /*!
   * @brief Minimal constructor initializing the depth data and the camera used for acquisition of the data.
   * @param data Depth data as CV_32F with invalid depth as -1 and positive in z-direction
   * @param cam Camera model during reconstruction process. Used for future reprojection of the depth into a point cloud
   */
  explicit Depthmap(const cv::Mat &data, const camera::Pinhole &cam);

  ~Depthmap() = default;
  Depthmap(const Depthmap& other) = default;
  Depthmap& operator=(const Depthmap& other) = default;

  /*!
   * @brief Getter for the camera model. Currently only pinhole model is supported.
   * @return Camera model used during reconstruction of the depth data
   */
  camera::Pinhole::ConstPtr getCamera() const;

  /*!
   * @brief Getter for the minimum scene depth in the provided depth map.
   * @return Smallest depth value of all valid (>0) pixels. Will be computed by a call to updateDepthParameters().
   */
  double getMinDepth() const;

  /*!
   * @brief Getter for the maximum scene depth in the provided depth map.
   * @return Largest depth value of all valid (>0) pixels. Will be computed by a call to updateDepthParameters().
   */
  double getMaxDepth() const;

  /*!
   * @brief Getter for the median scene depth in the provided depth map.
   * @return Median depth value of all valid (>0) pixels. Will be computed by a call to updateDepthParameters().
   */
  double getMedianDepth() const;

  /*!
   * @brief Computes the scene depth parameters by creating a vector of depth values, sorting them and extract the
   * relevant information. Note: Sorting all depth values can take substantial CPU load, which is why this function
   * must explicitly called - or not.
   */
  void updateDepthParameters();

  /*!
   * @brief Getter for the depth data. Matrix should be of type CV_32F with invalid depth set to -1.0 and positive in
   * z-direction of the camera. Returned by reference, so avoid screwing around with the data.
   * @return Reference to the depth data
   */
  cv::Mat& data();

private:

  /// Minimum depth value in the data matrix
  double _min_depth;

  /// Median depth value in the data matrix
  double _med_depth;

  /// Maximum depth value in the data matrix
  double _max_depth;

  /// Depth data as CV_32F floating point matrix. Invalid depth is set to -1.0.
  cv::Mat _data;

  /// Camera model during reconstruction. Most importantly is resized to fit the depth map size.
  camera::Pinhole::Ptr _cam;
};

}

#endif //OPENREALM_DEPTHMAP_H
