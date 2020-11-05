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

#include <realm_core/depthmap.h>

using namespace realm;

Depthmap::Depthmap(const cv::Mat &data, const camera::Pinhole &cam)
 : _data(data),
   _cam(std::make_shared<camera::Pinhole>(cam))
{
  if (_data.type() != CV_32F)
    throw(std::invalid_argument("Error creating depth map: Matrix type not CV_32F"));
  if (data.rows != _cam->height() || data.cols != _cam->width())
    throw(std::invalid_argument("Error creating depth map: Dimension mismatch! Camera size does not match data."));

  updateDepthParameters();
}

camera::Pinhole::ConstPtr Depthmap::getCamera() const
{
  return _cam;
}

double Depthmap::getMinDepth() const
{
  return _min_depth;
}

double Depthmap::getMaxDepth() const
{
  return _max_depth;
}

double Depthmap::getMedianDepth() const
{
  return _med_depth;
}

cv::Mat& Depthmap::data()
{
  return _data;
}

void Depthmap::updateDepthParameters()
{
  // First grab min and max values
  cv::Mat mask = (_data > 0);
  cv::minMaxLoc(_data, &_min_depth, &_max_depth, nullptr, nullptr, mask);

  std::vector<float> array;
  if (_data.isContinuous()) {
    array.assign((float*)_data.data, (float*)_data.data + _data.total()*_data.channels());
  } else {
    for (int i = 0; i < _data.rows; ++i) {
      array.insert(array.end(), _data.ptr<float>(i), _data.ptr<float>(i)+_data.cols*_data.channels());
    }
  }

  // Invalid depth values are set to -1.0
  std::vector<float> array_masked;
  std::copy_if(array.begin(), array.end(), back_inserter(array_masked),[](float n){ return  n > 0.0;});

  // Find median by sorting the array and get middle value
  std::sort(array.begin(), array.end());
  _med_depth = array[array.size()/2];
}