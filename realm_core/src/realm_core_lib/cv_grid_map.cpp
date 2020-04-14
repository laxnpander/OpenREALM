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
#include <cmath>
#include <opencv2/highgui.hpp>

#include <realm_core/camera.h>
#include <realm_core/structs.h>
#include <realm_core/enums.h>

#include <realm_core/cv_grid_map.h>

using namespace realm;

CvGridMap::CvGridMap()
    : _resolution(0.0)
{
}

CvGridMap::CvGridMap(const std::vector<std::string> &layer_names)
    : _resolution(0.0)
{
  // Add layer can not be used, because it checks for initialization
  // Layers are therefore directly pushed to the vector -> might not be the best solution
  for (const auto &layer : layer_names)
    _layers.push_back(Layer{layer, cv::Mat(), CV_INTER_LINEAR});
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

void CvGridMap::add(const Layer &layer)
{
  checkInit();
  checkValid(layer);
  // Add data if layer already exists, push to container if not
  if (!exists(layer.name))
    _layers.push_back(layer);
  else
  {
    _layers[findContainerIdx(layer.name)] = layer;
  }
}

void CvGridMap::add(const std::string &layer_name, const cv::Mat &layer_data)
{
  add(Layer{layer_name, layer_data, CV_INTER_LINEAR});
}

void CvGridMap::add(const std::string &layer_name, const cv::Mat &layer_data, int interpolation)
{
  add(Layer{layer_name, layer_data, interpolation});
}

void CvGridMap::add(const CvGridMap &submap, int flag_overlap_handle, bool do_extend)
{
  // Check if extending the map is necessary
  bool contains_sub_roi = containsRoi(submap._roi);
  // Extend if allowed and necessary
  if (do_extend && !contains_sub_roi)
  {
    extend(submap._roi);
    contains_sub_roi = true;
  }
  // Define copy area for layers by idx
  cv::Point2d pt_ulc(submap._roi.x, submap._roi.y);
  cv::Point2d pt_lrc(submap._roi.x + submap._roi.width, submap._roi.y - submap._roi.height);

  // overriding data in existing layers
  if (!contains_sub_roi) // only if not extending
  {
    if (submap._roi.x < _roi.x)
      pt_ulc.x = _roi.x;
    if (submap._roi.x+submap._roi.width > _roi.x+_roi.width)
      pt_lrc.x = _roi.x+_roi.width;
    if (submap._roi.y > _roi.y)
      pt_ulc.y = _roi.y;
    if (submap._roi.y-submap._roi.height < _roi.y-_roi.height)
      pt_lrc.y = _roi.y-_roi.height;
  }

  // getting index of grid roi
  cv::Point2i dst_idx_ulc = atIndex(pt_ulc);
  cv::Point2i dst_idx_lrc = atIndex(pt_lrc);
  cv::Point2i src_idx_ulc = submap.atIndex(pt_ulc);
  cv::Point2i src_idx_lrc = submap.atIndex(pt_lrc);
  cv::Rect2i dst_roi(dst_idx_ulc.x, dst_idx_ulc.y, dst_idx_lrc.x - dst_idx_ulc.x, dst_idx_lrc.y - dst_idx_ulc.y);

  // iterate through submap
  for (const auto &layer : submap._layers)
  {
    // Add of submap to this, if not existing
    if (!exists(layer.name))
      add(layer.name, cv::Mat());

    // Now layers will exist, get it
    uint32_t idx_layer = findContainerIdx(layer.name);

    // But might be empty
    if (_layers[idx_layer].data.empty())
    {
      if (layer.data.type() == CV_32FC1)
        _layers[idx_layer].data = cv::Mat::ones(_size, layer.data.type())*consts::getNoValue<float>();
      else if (layer.data.type() == CV_64FC1)
        _layers[idx_layer].data = cv::Mat::ones(_size, layer.data.type())*consts::getNoValue<double>();
      else
        _layers[idx_layer].data = cv::Mat::zeros(_size, layer.data.type());
    }

    // Get the data in the overlapping area of both mat
    cv::Mat src_data_roi = layer.data.rowRange(src_idx_ulc.y, src_idx_lrc.y).colRange(src_idx_ulc.x, src_idx_lrc.x);
    cv::Mat dst_data_roi = _layers[idx_layer].data(dst_roi);

    // Now is finally the turn to calculate overlap result and copy it to src grid map
    cv::Mat overlap;
    addMat(src_data_roi, dst_data_roi, overlap, flag_overlap_handle);
    overlap.copyTo(_layers[idx_layer].data(dst_roi));
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
  return (roi.x >= _roi.x && roi.x+roi.width <= _roi.x+_roi.width
      && roi.y <= _roi.y && roi.y-roi.height >= _roi.y-_roi.height);
}

cv::Mat CvGridMap::get(const std::string& layer_name)
{
  for (const auto &layer : _layers)
  {
    if (layer.name == layer_name)
      return layer.data;
  }
  throw std::out_of_range("No layer with name '" + layer_name + "' available.");
}

cv::Mat CvGridMap::get(const std::string& layer_name) const
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
  // Check if overlap exists
  if (other_map._roi.x + other_map._roi.width < _roi.x
      || other_map._roi.x > _roi.x + _roi.width
      || other_map._roi.y - other_map._roi.height > _roi.y
      || other_map._roi.y < _roi.y - _roi.y - _roi.height)
    return Overlap{nullptr, nullptr};

  // Define copy area for layers by idx
  cv::Point2d pt_ulc(other_map._roi.x, other_map._roi.y);
  cv::Point2d pt_lrc(other_map._roi.x + other_map._roi.width, other_map._roi.y - other_map._roi.height);

  // Get the roi of the overlap
  if (other_map._roi.x < _roi.x)
    pt_ulc.x = _roi.x;
  if (other_map._roi.x+other_map._roi.width > _roi.x+_roi.width)
    pt_lrc.x = _roi.x+_roi.width;
  if (other_map._roi.y > _roi.y)
    pt_ulc.y = _roi.y;
  if (other_map._roi.y-other_map._roi.height < _roi.y-_roi.height)
    pt_lrc.y = _roi.y-_roi.height;

  // create world frame roi
  cv::Rect2d roi_wf(pt_ulc.x, pt_ulc.y, pt_lrc.x-pt_ulc.x, pt_ulc.y-pt_lrc.y);

  // getting grid roi from both other and this map
  cv::Point2i this_idx_ulc = atIndex(pt_ulc);
  cv::Point2i this_idx_lrc = atIndex(pt_lrc);
  cv::Point2i other_idx_ulc = other_map.atIndex(pt_ulc);
  cv::Point2i other_idx_lrc = other_map.atIndex(pt_lrc);
  cv::Rect2i this_grid_roi(this_idx_ulc.x, this_idx_ulc.y, this_idx_lrc.x - this_idx_ulc.x, this_idx_lrc.y - this_idx_ulc.y);
  cv::Rect2i other_grid_roi(other_idx_ulc.x, other_idx_ulc.y, other_idx_lrc.x - other_idx_ulc.x, other_idx_lrc.y - other_idx_ulc.y);

  // Create this map as reference and initialize with layer names
  auto map_ref = std::make_shared<CvGridMap>();
  map_ref->setGeometry(roi_wf, _resolution);
  for (const auto &layer : _layers)
  {
    cv::Mat overlap_data = layer.data(this_grid_roi).clone();
    map_ref->add(layer.name, overlap_data, layer.interpolation);
  }

  // Create other map as added map and initialize with other map layer names
  auto map_added = std::make_shared<CvGridMap>();
  map_added->setGeometry(roi_wf, _resolution);
  for (const auto &layer : other_map._layers)
  {
    cv::Mat overlap_data = layer.data(other_grid_roi).clone();
    map_added->add(layer.name, overlap_data, layer.interpolation);
  }
  return std::make_pair(map_ref, map_added);
}

cv::Mat CvGridMap::operator[](const std::string& layer_name)
{
  return get(layer_name);
}

cv::Mat CvGridMap::operator[](const std::string& layer_name) const
{
  return get(layer_name);
}

void CvGridMap::setGeometry(const cv::Rect2d &roi, const double &resolution)
{
  assert(roi.width > 0.0);
  assert(roi.height > 0.0);
  assert(resolution > 0.0);
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

void CvGridMap::extend(const cv::Rect2d &roi)
{
  assert(roi.width > 0 && roi.height > 0);  // false if roi empty -> function called, without valid argument
  checkInit();
  // keep track if dimensions changed
  bool changed_roi = false;

  // dimension change of world frame roi
  double roi_x_minus = 0.0;
  double roi_y_minus = 0.0;
  double roi_x_plus = 0.0;
  double roi_y_plus = 0.0;

  // size change corresponding to the roi change
  int size_x_minus = 0;
  int size_y_minus = 0;
  int size_x_plus = 0;
  int size_y_plus = 0;

  // compute boundaries of new roi
  if (roi.x < _roi.x)
  {
    roi_x_minus = _roi.x - roi.x;
    changed_roi = true;
  }
  if (roi.x+roi.width > _roi.x+_roi.width)
  {
    roi_x_plus = (roi.x+roi.width) - (_roi.x+_roi.width);
    changed_roi = true;
  }
  if (roi.y > _roi.y)
  {
    roi_y_plus = roi.y - _roi.y;
    changed_roi = true;
  }
  if (roi.y-roi.height < _roi.y-_roi.height)
  {
    roi_y_minus = (_roi.y-_roi.height) - (roi.y-roi.height);
    changed_roi = true;
  }

  // did boundaries change?
  if (changed_roi)
  {
    cv::Rect2d roi_desired;
    roi_desired.x = _roi.x - roi_x_minus;
    roi_desired.y = _roi.y + roi_y_plus;
    roi_desired.width = _roi.width + roi_x_plus + roi_x_minus;
    roi_desired.height = _roi.height + roi_y_plus + roi_y_minus;
    // Roi has changed to a new desired roi -> fit the desired roi
    // and corresponding size of the grid to the resolution that was set
    cv::Rect2d roi_set;
    fitGeometryToResolution(roi_desired, roi_set, _size);
    // Add the grid growth to existing layer data
    size_x_plus = static_cast<int>(std::round(((roi_set.x+roi_set.width) - (_roi.x+_roi.width))/_resolution));
    size_y_plus = static_cast<int>(std::round(((_roi.y-_roi.height) - (roi_set.y-roi_set.height))/_resolution));
    size_x_minus = static_cast<int>(std::round((_roi.x - roi_set.x)/_resolution));
    size_y_minus = static_cast<int>(std::round((roi_set.y - _roi.y)/_resolution));
    _roi = roi_set;
    // afterwards add new size to existing layers
    for (auto &layer : _layers)
      if (!layer.data.empty())
      {
        if(layer.data.type() == CV_32F)
          cv::copyMakeBorder(layer.data, layer.data, size_y_minus, size_y_plus, size_x_minus, size_x_plus, cv::BORDER_CONSTANT, consts::getNoValue<float>());
        else if (layer.data.type() == CV_64F)
          cv::copyMakeBorder(layer.data, layer.data, size_y_minus, size_y_plus, size_x_minus, size_x_plus, cv::BORDER_CONSTANT, consts::getNoValue<double>());
        else
          cv::copyMakeBorder(layer.data, layer.data, size_y_minus, size_y_plus, size_x_minus, size_x_plus, cv::BORDER_CONSTANT);
      }
  }
}

void CvGridMap::changeResolution(const double &resolution)
{
  _resolution = resolution;
  fitGeometryToResolution(_roi, _roi, _size);
  for (auto &layer : _layers)
    if (!layer.data.empty() && (layer.data.cols != _size.width || layer.data.rows != _size.height))
      cv::resize(layer.data, layer.data, _size, layer.interpolation);
}

cv::Point2i CvGridMap::atIndex(const cv::Point2d &pos) const
{
  if (pos.x < _roi.x-_resolution/2 || pos.x > _roi.x+_roi.width+_resolution/2
      || pos.y > _roi.y+_resolution/2 || pos.y < _roi.y-_roi.height-_resolution/2)
    throw std::out_of_range("Requested world position is out of bounds.");
  cv::Point2i idx;
  idx.x = static_cast<int>(std::round((pos.x -_roi.x)/_resolution));
  idx.y = static_cast<int>(std::round((_roi.y - pos.y)/_resolution));
  return idx;
}

cv::Point2d CvGridMap::atPosition2d(uint32_t r, uint32_t c) const
{
  if (r > _size.height || r < 0)
    std::cout << "r: " << r << ", c: " << c << " max: " << _size.height << std::endl;

  // check validity
  assert(r < _size.height && r >= 0);
  assert(c < _size.width && c >= 0);
  assert(_roi.width > 0 && _roi.height > 0);
  assert(_resolution > 0.0);

  // calculate position of grid element centroid
  cv::Point2d pos;
  pos.x = (_roi.x + _resolution/2)+(double)c*_resolution;
  pos.y = (_roi.y - _resolution/2)-(double)r*_resolution;
  return pos;
}

cv::Point3d CvGridMap::atPosition3d(const int &r, const int &c, const std::string &layer_name) const
{
  cv::Mat layer_data = get(layer_name);
  // check validity
  assert(r < _size.height && r >= 0);
  assert(c < _size.width && c >= 0);
  assert(_roi.width > 0 && _roi.height > 0);
  assert(_resolution > 0.0);
  assert(!layer_data.empty());
  // create position and set data
  cv::Point3d pos;
  pos.x = (_roi.x + _resolution/2)+(double)c*_resolution;
  pos.y = (_roi.y - _resolution/2)-(double)r*_resolution;
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

void CvGridMap::checkValid(const cv::Mat &data)
{
  assert(data.empty() || (data.cols == _size.width && data.rows == _size.height));
  assert(data.empty() || data.type() == CV_32FC1 || data.type() == CV_32FC3 || data.type() == CV_64F || data.type() == CV_8UC1
             || data.type() == CV_8UC2 || data.type() == CV_8UC3 || data.type() == CV_8UC4 || CV_16UC1);
}

void CvGridMap::checkValid(const Layer &layer)
{
  checkValid(layer.data);
}

void CvGridMap::checkInit()
{
  assert(_size.width > 0 && _size.height > 0);
  assert(_roi.width > 0 && _roi.height > 0);
  assert(_resolution > 0.0);
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
  assert(_resolution > 0.0);
  size_set.width = static_cast<int>(std::round(roi_desired.width/_resolution));
  size_set.height = static_cast<int>(std::round(roi_desired.height/_resolution));
  // Correct roi, because width and height were changed according to the resolution
  roi_set.width = _resolution * static_cast<double>(size_set.width);
  roi_set.height = _resolution * static_cast<double>(size_set.height);
  roi_set.x = roi_desired.x - (roi_set.width - roi_desired.width) / 2.0;
  roi_set.y = roi_desired.y + (roi_set.height - roi_desired.height) / 2.0;
}