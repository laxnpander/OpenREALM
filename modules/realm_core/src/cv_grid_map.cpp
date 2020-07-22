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
#include <algorithm>
#include <cmath>

#include <realm_core/loguru.h>
#include <realm_core/cv_grid_map.h>

using namespace realm;

CvGridMap::CvGridMap()
    : _resolution(1.0)
{
}

CvGridMap::CvGridMap(const cv::Rect2d &roi, double resolution)
    : _resolution(resolution)
{
  setGeometry(roi, _resolution);
}

CvGridMap CvGridMap::clone()
{
  CvGridMap copy;
  copy.setGeometry(_roi, _resolution);
  for (const auto &layer : _layers)
    copy.add(layer.name, layer.data.clone(), layer.interpolation);
  return copy;
}

CvGridMap CvGridMap::cloneSubmap(const std::vector<std::string> &layer_names)
{
  CvGridMap copy;
  copy.setGeometry(_roi, _resolution);
  for (const auto &layer_name : layer_names)
  {
    Layer layer = getLayer(layer_name);
    copy.add(layer.name, layer.data.clone(), layer.interpolation);
  }
  return copy;
}

void CvGridMap::add(const Layer &layer, bool is_data_empty)
{
  if (!is_data_empty && (layer.data.size().width != _size.width || layer.data.size().height != _size.height))
  {
    throw(std::invalid_argument("Error: Adding Layer failed. Size of data does not match grid map size."));
  }

  // Add data if layer already exists, push to container if not
  if (!exists(layer.name))
    _layers.push_back(layer);
  else
  {
    _layers[findContainerIdx(layer.name)] = layer;
  }
}

void CvGridMap::add(const std::string &layer_name, const cv::Mat &layer_data, int interpolation)
{
  add(Layer{layer_name, layer_data, interpolation});
}

void CvGridMap::add(const CvGridMap &submap, int flag_overlap_handle, bool do_extend)
{
  if (fabs(resolution() - submap.resolution()) > std::numeric_limits<double>::epsilon())
    throw(std::invalid_argument("Error add submap: Resolution mismatch!"));

  // Check if extending the map is necessary
  bool contains_sub_roi = containsRoi(submap._roi);

  // Create the theoretical ROI bounds to copy the submap data to
  cv::Rect2d copy_roi = submap.roi();

  // If extension is allowed, then extend the reference map first
  if (do_extend && !contains_sub_roi)
  {
    extendToInclude(copy_roi);
  }
  // If extension is not allowed and the boundaries of the submap are outside the reference map, adjust the copy area
  else if(!do_extend && !contains_sub_roi)
  {
    // Get the overlapping region of interest of both rectangles
    copy_roi = (copy_roi & _roi);

    // Check if there is an overlap at all
    if (copy_roi.area() < 10e-6)
    {
      LOG_F(WARNING, "Called 'add(Submap)' on submap outside the reference image without allowing to extend it.");
      return;
    }
  }
  // getting index of grid roi
  cv::Rect2i src_roi(submap.atIndexROI(copy_roi));
  cv::Rect2i dst_roi(this->atIndexROI(copy_roi));

  // iterate through submap
  for (const auto &submap_layer : submap._layers)
  {
    // Add of submap to this, if not existing
    if (!exists(submap_layer.name))
      add(Layer{submap_layer.name, cv::Mat()}, true);

    // Now layers will exist, get it
    uint32_t idx_layer = findContainerIdx(submap_layer.name);

    // But might be empty
    if (_layers[idx_layer].data.empty())
    {
      switch(_layers[idx_layer].data.type())
      {
        case CV_32F:
          _layers[idx_layer].data = cv::Mat(_size, submap_layer.data.type(), std::numeric_limits<float>::quiet_NaN());
          break;
        case CV_64F:
          _layers[idx_layer].data = cv::Mat(_size, submap_layer.data.type(), std::numeric_limits<double>::quiet_NaN());
          break;
        default:
          _layers[idx_layer].data = cv::Mat::zeros(_size, submap_layer.data.type());
      }
    }

    // Get the data in the overlapping area of both mat
    cv::Mat src_data_roi = submap_layer.data(src_roi);
    cv::Mat dst_data_roi = _layers[idx_layer].data(dst_roi);

    // Final check for matrix size
    if (src_data_roi.rows != dst_data_roi.rows || src_data_roi.cols != dst_data_roi.cols)
    {
      LOG_F(WARNING, "Overlap area could not be merged. Matrix dimensions mismatched!");
      continue;
    }

    // Now is finally the turn to calculate overlap result and copy it to src grid map
    mergeMatrices(src_data_roi, dst_data_roi, flag_overlap_handle);
    dst_data_roi.copyTo(_layers[idx_layer].data(dst_roi));
  }
}

bool CvGridMap::empty() const
{
  return _size.width == 0 || _size.height == 0;
}

bool CvGridMap::exists(const std::string &layer_name) const
{
  for (const auto &layer : _layers)
  {
    if (layer.name == layer_name)
      return true;
  }
  return false;
}

bool CvGridMap::containsRoi(const cv::Rect2d &roi) const
{
  return fabs((_roi & roi).area() - roi.area()) < 10e-6;
}

cv::Mat& CvGridMap::get(const std::string& layer_name)
{
  for (auto &layer : _layers)
  {
    if (layer.name == layer_name)
      return layer.data;
  }
  throw std::out_of_range("No layer with name '" + layer_name + "' available.");
}

const cv::Mat& CvGridMap::get(const std::string& layer_name) const
{
  for (const auto &layer : _layers)
  {
    if (layer.name == layer_name)
      return layer.data;
  }
  throw std::out_of_range("No layer with name '" + layer_name + "' available.");
}

CvGridMap::Layer CvGridMap::getLayer(const std::string& layer_name) const
{
  for (const auto& layer : _layers)
    if (layer.name == layer_name)
      return layer;
}

std::vector<std::string> CvGridMap::getAllLayerNames() const
{
  std::vector<std::string> layer_names;
  for (const auto &layer : _layers)
    layer_names.push_back(layer.name);
  return layer_names;
}

CvGridMap CvGridMap::getSubmap(const std::vector<std::string> &layer_names) const
{
  CvGridMap submap;
  submap.setGeometry(_roi, _resolution);
  for (const auto &layer_name : layer_names)
  {
    Layer layer = getLayer(layer_name);
    submap.add(layer.name, layer.data, layer.interpolation);
  }
  return submap;
}

CvGridMap CvGridMap::getSubmap(const std::vector<std::string> &layer_names, const cv::Rect2d &roi) const
{
  CvGridMap submap;
  submap.setGeometry(roi, _resolution);
  CvGridMap::Overlap overlap = getOverlap(submap);

  if (overlap.first != nullptr && overlap.second != nullptr)
    return overlap.first->getSubmap(layer_names);
  else
    throw(std::out_of_range("Error extracting submap: No overlap!"));
}

CvGridMap::Overlap CvGridMap::getOverlap(const CvGridMap &other_map) const
{
  cv::Rect2d overlap_roi = (_roi & other_map._roi);

  // Check if overlap exists
  if (overlap_roi.area() < 10e-6)
    return Overlap{nullptr, nullptr};

  // getting matrix roi from both other and this map
  cv::Rect2i this_grid_roi(this->atIndexROI(overlap_roi));
  cv::Rect2i other_grid_roi(other_map.atIndexROI(overlap_roi));

  // Create this map as reference and initialize with layer names
  auto map_ref = std::make_shared<CvGridMap>();
  map_ref->setGeometry(overlap_roi, _resolution);
  for (const auto &layer : _layers)
  {
    cv::Mat overlap_data = layer.data(this_grid_roi).clone();
    map_ref->add(layer.name, overlap_data, layer.interpolation);
  }

  // Create other map as added map and initialize with other map layer names
  auto map_added = std::make_shared<CvGridMap>();
  map_added->setGeometry(overlap_roi, _resolution);
  for (const auto &layer : other_map._layers)
  {
    cv::Mat overlap_data = layer.data(other_grid_roi).clone();
    map_added->add(layer.name, overlap_data, layer.interpolation);
  }
  return std::make_pair(map_ref, map_added);
}

cv::Mat& CvGridMap::operator[](const std::string& layer_name)
{
  return get(layer_name);
}

const cv::Mat& CvGridMap::operator[](const std::string& layer_name) const
{
  return get(layer_name);
}

void CvGridMap::setGeometry(const cv::Rect2d &roi, double resolution)
{
  if (roi.width < 10e-6 || roi.height < 10e-6)
    LOG_F(WARNING, "Grid dimension: %f x %f", roi.width, roi.height);
  if (_resolution < 10e-6)
    throw(std::invalid_argument("Error: Resolution is zero!"));

  // Set basic members
  _resolution = resolution;
  fitGeometryToResolution(roi, _roi, _size);

  // Release all current data
  for (auto &layer : _layers)
    layer.data.release();
}

void CvGridMap::setLayerInterpolation(const std::string& layer_name, int interpolation)
{
  _layers[findContainerIdx(layer_name)].interpolation = interpolation;
}

void CvGridMap::extendToInclude(const cv::Rect2d &roi)
{
  if (roi.width <= 0.0 || roi.height <= 0.0)
    throw(std::invalid_argument("Error: Extending grid to include ROI failed. ROI dimensions zero!"));

  cv::Rect2d bounding_box;
  bounding_box.x = std::min({_roi.x, roi.x});
  bounding_box.y = std::min({_roi.y, roi.y});
  bounding_box.width = std::max({_roi.x + _roi.width, roi.x + roi.width}) - bounding_box.x;
  bounding_box.height = std::max({_roi.y + _roi.height, roi.y + roi.height}) - bounding_box.y;

  // The bounding box is the new ROI, next fit it to the given resolution and adjust the corresponding size of the grid
  cv::Rect2d roi_set;
  fitGeometryToResolution(bounding_box, roi_set, _size);

  // Add the grid growth to existing layer data
  int size_x_right  = static_cast<int>(std::round((roi_set.x+roi_set.width - (_roi.x+_roi.width)) / _resolution));
  int size_y_top    = static_cast<int>(std::round((roi_set.y+roi_set.height - (_roi.y+_roi.height)) / _resolution));
  int size_x_left   = static_cast<int>(std::round((_roi.x - roi_set.x) / _resolution));
  int size_y_bottom = static_cast<int>(std::round((_roi.y - roi_set.y) / _resolution));

  if (size_x_left < 0) size_x_left = 0;
  if (size_y_bottom < 0) size_y_bottom = 0;
  if (size_x_right < 0) size_x_right = 0;
  if (size_y_top < 0) size_y_top = 0;

  _roi = roi_set;

  // afterwards add new size to existing layers
  for (auto &layer : _layers)
    if (!layer.data.empty())
    {
      switch(layer.data.type())
      {
        case CV_32F:
          cv::copyMakeBorder(layer.data, layer.data, size_y_top, size_y_bottom, size_x_left, size_x_right, cv::BORDER_CONSTANT, std::numeric_limits<float>::quiet_NaN());
          break;
        case CV_64F:
          cv::copyMakeBorder(layer.data, layer.data, size_y_top, size_y_bottom, size_x_left, size_x_right, cv::BORDER_CONSTANT, std::numeric_limits<double>::quiet_NaN());
          break;
        default:
          cv::copyMakeBorder(layer.data, layer.data, size_y_top, size_y_bottom, size_x_left, size_x_right, cv::BORDER_CONSTANT);
      }
    }
}

void CvGridMap::changeResolution(double resolution)
{
  _resolution = resolution;
  fitGeometryToResolution(_roi, _roi, _size);
  for (auto &layer : _layers)
    if (!layer.data.empty() && (layer.data.cols != _size.width || layer.data.rows != _size.height))
      cv::resize(layer.data, layer.data, _size, layer.interpolation);
}

cv::Point2i CvGridMap::atIndex(const cv::Point2d &pos) const
{
  double x_rounded = roundToResolution(pos.x, _resolution);
  double y_rounded = roundToResolution(pos.y, _resolution);

  double epsilon = _resolution / 2;
  if (x_rounded < _roi.x-epsilon || x_rounded > _roi.x+_roi.width+epsilon
          || y_rounded < _roi.y-epsilon || y_rounded > _roi.y+_roi.height+epsilon)
    throw std::out_of_range("Requested world position is out of bounds.");

  cv::Point2i idx;
  idx.x = static_cast<int>(std::round((x_rounded -_roi.x)/_resolution));
  idx.y = static_cast<int>(std::round((_roi.y + _roi.height - y_rounded)/_resolution));

  return idx;
}

cv::Rect2i CvGridMap::atIndexROI(const cv::Rect2d &roi) const
{
  // Get the four corners of the world coordinates in the matrix
  cv::Point2i idx = atIndex(cv::Point2d(roi.x, roi.y+roi.height));

  int width = static_cast<int>(std::round(roi.width/_resolution)) + 1;
  int height = static_cast<int>(std::round(roi.height/_resolution)) + 1;

  return cv::Rect2i(idx.x, idx.y, width, height);
}

cv::Point2d CvGridMap::atPosition2d(uint32_t r, uint32_t c) const
{
  // check validity
  assert(r < _size.height && r >= 0);
  assert(c < _size.width && c >= 0);
  assert(_roi.width > 0 && _roi.height > 0);
  assert(_resolution > 0.0);

  cv::Point2d pos;
  pos.x = _roi.x + static_cast<double>(c)*_resolution;
  pos.y = _roi.y + _roi.height - static_cast<double>(r)*_resolution;  // ENU world frame
  return pos;
}

cv::Point3d CvGridMap::atPosition3d(const int &r, const int &c, const std::string &layer_name) const
{
  cv::Mat layer_data = get(layer_name);

  // check validity
  if (layer_data.empty())
    throw(std::runtime_error("Error: Layer data empty! Requesting data failed."));
  if (r < 0 || r >= _size.height || c < 0 || c >= _size.width)
    throw(std::invalid_argument("Error: Requested position outside matrix boundaries!"));

  // create position and set data
  cv::Point3d pos;
  pos.x = _roi.x + static_cast<double>(c)*_resolution;
  pos.y = _roi.y + _roi.height - static_cast<double>(r)*_resolution;  // ENU world frame

  if (layer_data.type() == CV_32F)
    pos.z = static_cast<double>(layer_data.at<float>(r, c));
  else if (layer_data.type() == CV_64F)
    pos.z = layer_data.at<double>(r, c);
  else
    throw(std::out_of_range("Error accessing 3d position in CvGridMap: z-coordinate data type not supported."));
  return pos;
}

double CvGridMap::resolution() const
{
  return _resolution;
}

cv::Size2i CvGridMap::size() const
{
  return _size;
}

cv::Rect2d CvGridMap::roi() const
{
  return _roi;
}

void CvGridMap::printInfo() const
{
  std::cout.precision(10);
  std::cout << "##### CvGridMap Debug Info #####" << std::endl;
  std::cout << "Geometry:" << std::endl;
  std::cout << "- Roi: " << _roi << std::endl;
  std::cout << "- Size: " << _size.width << "x" << _size.height << std::endl;
  std::cout << "Layers:" << std::endl;
  for (const auto &layer : _layers)
    std::cout << "- ['" << layer.name << "']: size = " << layer.data.cols << "x" << layer.data.rows << std::endl;
}

void CvGridMap::mergeMatrices(const cv::Mat &from, cv::Mat &to, int flag_merge_handling)
{
  switch(flag_merge_handling)
  {
    case REALM_OVERWRITE_ALL:
      to = from;
      break;
    case REALM_OVERWRITE_ZERO:
      cv::Mat mask;
      if (to.type() == CV_32F || to.type() == CV_64F)
        mask = (to != to) & (from == from);
      else
        mask = (to == 0) & (from > 0);
      from.copyTo(to, mask);
      break;
  }
}

void CvGridMap::checkValid(const cv::Mat &data)
{
  if (data.empty())
    throw(std::invalid_argument("Error: Layer data empty!"));
  if (data.cols != _size.width || data.rows != _size.height)
    throw(std::invalid_argument("Error: Layer dimension mismatch!"));
  if (!isMatrixTypeValid(data.type()))
    throw(std::invalid_argument("Error: Layer data type not supported!"));
}

void CvGridMap::checkValid(const Layer &layer)
{
  checkValid(layer.data);
}

bool CvGridMap::isMatrixTypeValid(int type)
{
  switch (type)
  {
    case CV_32F:
      return true;
    case CV_64F:
      return true;
    case CV_8U:
      return true;
    case CV_16U:
      return true;
    default:
      return false;
  }
}

uint32_t CvGridMap::findContainerIdx(const std::string &layer_name)
{
  for (uint32_t i = 0; i < _layers.size(); ++i)
    if (_layers[i].name == layer_name)
      return i;
  throw(std::out_of_range("Error: Index for layer not found!"));
}

void CvGridMap::fitGeometryToResolution(const cv::Rect2d &roi_desired, cv::Rect2d &roi_set, cv::Size2i &size_set)
{
  // We round the geometry of our region of interest to fit exactly into our resolution. So position x,y and dimensions
  // width,height are always multiples of the resolution
  roi_set.x = roundToResolution(roi_desired.x, _resolution);
  roi_set.y = roundToResolution(roi_desired.y, _resolution);
  roi_set.width = roundToResolution(roi_desired.width, _resolution);
  roi_set.height = roundToResolution(roi_desired.height, _resolution);

  // Please note, that the grid is always 1 element bigger than the width and height of the ROI. This is because the
  // samples in the world coordinate frame are in the center of the matrix elements. Therefore a resolution/2 is added
  // to all sides of the grid resulting in the additional + 1 as matrix size.
  size_set.width = static_cast<int>(std::round(roi_set.width/_resolution))+1;
  size_set.height = static_cast<int>(std::round(roi_set.height/_resolution))+1;
}

double CvGridMap::roundToResolution(double value, double resolution)
{
  double remainder = fmod(value, resolution);
  if (fabs(remainder) < 10e-6)
    return value;
  else
    return value + resolution - remainder;
}