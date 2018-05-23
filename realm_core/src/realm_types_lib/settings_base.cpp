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

#include <realm_types/settings_base.h>

using namespace realm;

// PUBLIC

void SettingsBase::set(const std::string &param_name, int val)
{
  set(_params_i, {param_name, val});
}

void SettingsBase::set(const std::string &param_name, double val)
{
  set(_params_d, {param_name, val});
}

void SettingsBase::set(const std::string &param_name, const std::string &val)
{
  set(_params_s, {param_name, val});
}

void SettingsBase::loadFromFile(const std::string &filepath)
{
  cv::FileStorage fs(filepath, cv::FileStorage::READ);
  if (fs.isOpened())
  {
    for (auto &param : _params_i)
      fs[param.first] >> param.second.value;
    for (auto &param : _params_d)
      fs[param.first] >> param.second.value;
    for (auto &param : _params_s)
      fs[param.first] >> param.second.value;
  }
  else
    throw std::out_of_range("Error. Could not load settings from file: " + filepath);
}

bool SettingsBase::has(const std::string &param_name)
{
  for (auto &param : _params_i)
    if (param.first == param_name)
      return true;
  for (auto &param : _params_d)
    if (param.first == param_name)
      return true;
  for (auto &param : _params_s)
    if (param.first == param_name)
      return true;
  return false;
}

void SettingsBase::print()
{
  std::cout.precision(6);
  for (auto &param : _params_i)
    std::cout << "\t<Param>[" << param.first << "]: " << param.second.value << std::endl;
  for (auto &param : _params_d)
    std::cout << "\t<Param>[" << param.first << "]: " << param.second.value << std::endl;
  for (auto &param : _params_s)
    std::cout << "\t<Param>[" << param.first << "]: " << param.second.value << std::endl;
}

// PROTECTED

void SettingsBase::add(const std::string &key, const Parameter_t<int> &param)
{
  add(_params_i, {key, param});
}

void SettingsBase::add(const std::string &key, const Parameter_t<double> &param)
{
  add(_params_d, {key, param});
}

void SettingsBase::add(const std::string &key, const Parameter_t<std::string> &param)
{
  add(_params_s, {key, param});
}

// PRIVATE

void SettingsBase::get(const std::string &key, Parameter_t<int> *param) const
{
  try
  {
    *param = _params_i.at(key);
  }
  catch(std::out_of_range &e)
  {
    throw(std::out_of_range("Error: Parameter with name " + key + " could not be found in settings."));
  }
}

void SettingsBase::get(const std::string &key, Parameter_t<double> *param) const
{
  try
  {
    *param = _params_d.at(key);
  }
  catch(std::out_of_range &e)
  {
    throw(std::out_of_range("Error: Parameter with name " + key + " could not be found in settings."));
  }
}

void SettingsBase::get(const std::string &key, Parameter_t<std::string> *param) const
{
  try
  {
    *param = _params_s.at(key);
  }
  catch(std::out_of_range &e)
  {
    throw(std::out_of_range("Error: Parameter with name " + key + " could not be found in settings."));
  }
}