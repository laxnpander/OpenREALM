

#include <realm_core/settings_base.h>

using namespace realm;

// PUBLIC

SettingsBase::Variant SettingsBase::operator[](const std::string &key) const
{
  return get(key);
}

SettingsBase::Variant SettingsBase::get(const std::string &key) const
{
  auto it = m_parameters.find(key);
  if (it != m_parameters.end())
    return *it->second;
  else
    throw(std::out_of_range("Error: Parameter with name '" + key + "' could not be found in settings."));
}

void SettingsBase::set(const std::string &key, int val)
{
  m_parameters[key]->m_int_container.value = val;
}

void SettingsBase::set(const std::string &key, double val)
{
  m_parameters[key]->m_double_container.value = val;
}

void SettingsBase::set(const std::string &key, const std::string &val)
{
  m_parameters[key]->m_string_container.value = val;
}

void SettingsBase::set(const std::string &key, const cv::Mat &val)
{
  m_parameters[key]->m_mat_container.value = val;
}

void SettingsBase::loadFromFile(const std::string &filepath)
{
  cv::FileStorage fs(filepath, cv::FileStorage::READ);
  if (fs.isOpened())
  {
    for (auto &param : m_parameters)
    {
      if (param.second->m_type == SettingsBase::Variant::VariantType::INT)    fs[param.first] >> param.second->m_int_container.value;
      if (param.second->m_type == SettingsBase::Variant::VariantType::DOUBLE) fs[param.first] >> param.second->m_double_container.value;
      if (param.second->m_type == SettingsBase::Variant::VariantType::STRING) fs[param.first] >> param.second->m_string_container.value;
      if (param.second->m_type == SettingsBase::Variant::VariantType::MATRIX) fs[param.first] >> param.second->m_mat_container.value;
    }
  }
  else
    throw std::out_of_range("Error. Could not load settings from file: " + filepath);
}

bool SettingsBase::has(const std::string &key) const
{
  auto it = m_parameters.find(key);
  if (it != m_parameters.end())
    return true;
  else
    return false;
}

void SettingsBase::print()
{
  std::cout.precision(6);
  for (auto &param : m_parameters)
  {
    if (param.second->m_type == SettingsBase::Variant::VariantType::INT)    std::cout << "\t<Param>[" << param.first << "]: " << param.second->toInt() << std::endl;
    if (param.second->m_type == SettingsBase::Variant::VariantType::DOUBLE) std::cout << "\t<Param>[" << param.first << "]: " << param.second->toDouble() << std::endl;
    if (param.second->m_type == SettingsBase::Variant::VariantType::STRING) std::cout << "\t<Param>[" << param.first << "]: " << param.second->toString() << std::endl;
    if (param.second->m_type == SettingsBase::Variant::VariantType::MATRIX) std::cout << "\t<Param>[" << param.first << "]: " << param.second->toMat() << std::endl;
  }
}

// PROTECTED

void SettingsBase::add(const std::string &key, const Parameter_t<int> &param)
{
  m_parameters[key].reset(new Variant(param));
}

void SettingsBase::add(const std::string &key, const Parameter_t<double> &param)
{
  m_parameters[key].reset(new Variant(param));
}

void SettingsBase::add(const std::string &key, const Parameter_t<std::string> &param)
{
  m_parameters[key].reset(new Variant(param));
}

void SettingsBase::add(const std::string &key, const Parameter_t<cv::Mat> &param)
{
  m_parameters[key].reset(new Variant(param));
}