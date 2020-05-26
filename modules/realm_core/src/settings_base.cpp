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

#include <realm_core/settings_base.h>

using namespace realm;

// PUBLIC

SettingsBase::Variant SettingsBase::operator[](const std::string &key) const
{
  return get(key);
}

SettingsBase::Variant SettingsBase::get(const std::string &key) const
{
  auto it = _parameters.find(key);
  if (it != _parameters.end())
    return *it->second;
  else
    throw(std::out_of_range("Error: Parameter with name '" + key + "' could not be found in settings."));
}

void SettingsBase::set(const std::string &key, int val)
{
  _parameters[key]->_int_container.value = val;
}

void SettingsBase::set(const std::string &key, double val)
{
  _parameters[key]->_double_container.value = val;
}

void SettingsBase::set(const std::string &key, const std::string &val)
{
  _parameters[key]->_string_container.value = val;
}

void SettingsBase::loadFromFile(const std::string &filepath)
{
  cv::FileStorage fs(filepath, cv::FileStorage::READ);
  if (fs.isOpened())
  {
    for (auto &param : _parameters) 
    {
      if (param.second->_type == SettingsBase::Variant::VariantType::INT) fs[param.first] >> param.second->_int_container.value;
      if (param.second->_type == SettingsBase::Variant::VariantType::DOUBLE) fs[param.first] >> param.second->_double_container.value;
      if (param.second->_type == SettingsBase::Variant::VariantType::STRING) fs[param.first] >> param.second->_string_container.value;
    }
  }
  else
    throw std::out_of_range("Error. Could not load settings from file: " + filepath);
}

bool SettingsBase::has(const std::string &key) const
{
  auto it = _parameters.find(key);
  if (it != _parameters.end())
    return true;
  else
    return false;
}

void SettingsBase::print()
{
  std::cout.precision(6);
  for (auto &param : _parameters)
  {
    if (param.second->_type == SettingsBase::Variant::VariantType::INT) std::cout << "\t<Param>[" << param.first << "]: " << param.second->toInt() << std::endl;
    if (param.second->_type == SettingsBase::Variant::VariantType::DOUBLE) std::cout << "\t<Param>[" << param.first << "]: " << param.second->toDouble() << std::endl;
    if (param.second->_type == SettingsBase::Variant::VariantType::STRING) std::cout << "\t<Param>[" << param.first << "]: " << param.second->toString() << std::endl;
  }
}

// PROTECTED

void SettingsBase::add(const std::string &key, const Parameter_t<int> &param)
{
  _parameters[key].reset(new Variant(param));
}

void SettingsBase::add(const std::string &key, const Parameter_t<double> &param)
{
  _parameters[key].reset(new Variant(param));
}

void SettingsBase::add(const std::string &key, const Parameter_t<std::string> &param)
{
  _parameters[key].reset(new Variant(param));
}