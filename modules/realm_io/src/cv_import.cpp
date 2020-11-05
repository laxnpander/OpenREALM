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

#include <realm_io/cv_import.h>
#include <realm_io/utilities.h>

using namespace realm;

cv::Mat io::loadImage(const std::string &filepath)
{
  if (!io::fileExists(filepath))
    throw(std::invalid_argument("Error loading image: File does not exist!"));

  std::string suffix = filepath.substr(filepath.size()-3, 3);

  if(suffix == "png" || suffix == "jpg")
    return cv::imread(filepath, cv::IMREAD_UNCHANGED);
  else if (suffix == "bin")
    return loadImageFromBinary(filepath);
  else
    throw(std::invalid_argument("Error writing image: Unknown suffix"));
}

cv::Mat io::loadImageFromBinary(const std::string &filepath)
{
  FILE* file = fopen(filepath.c_str(), "rb");

  int header[4];

  size_t elements_read;
  elements_read = fread(header, sizeof(int), 4, file);

  if (elements_read != 4)
    throw(std::runtime_error("Error reading binary: Elements read do not match matrix dimension!"));

  int cols               = header[0];
  int rows               = header[1];
  int elem_size_in_bytes = header[2];
  int elem_type          = header[3];

  cv::Mat data = cv::Mat::ones(rows, cols, elem_type);

  elements_read = fread(data.data, elem_size_in_bytes, (size_t)(cols * rows), file);

  if (elements_read != (size_t)(cols * rows))
    throw(std::runtime_error("Error reading binary: Elements read do not match matrix dimension!"));

  fclose(file);

  return data;
}