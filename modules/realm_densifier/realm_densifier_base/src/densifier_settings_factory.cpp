

#include <realm_densifier_base/densifier_settings_factory.h>

using namespace realm;

DensifierSettings::Ptr DensifierSettingsFactory::load(const std::string &filepath, const std::string &directory)
{
  std::string method = DensifierSettings::sneakParameterFromFile<std::string>("type", filepath);
  if (method == "DUMMY")
    return loadDefault<DensifierDummySettings>(filepath, directory);
  if (method == "PSL")
    return loadDefault<PlaneSweepSettings>(filepath, directory);
//if (method == "YOUR_IMPLEMENTATION")
//  return loadDefault<YOUR_IMPLEMENTATION_SETTINGS>(settings, directory, filename);
  throw (std::invalid_argument("Error: Loading densifier settings failed. Method '" + method + "' not recognized"));
}

template <typename T>
DensifierSettings::Ptr DensifierSettingsFactory::loadDefault(const std::string &filepath, const std::string &directory)
{
  auto settings = std::make_shared<T>();
  settings->loadFromFile(filepath);
  return settings;
}