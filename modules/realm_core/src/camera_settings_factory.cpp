

#include <realm_core/camera_settings_factory.h>

using namespace realm;

CameraSettings::Ptr CameraSettingsFactory::load(const std::string &filepath)
{
  // Identify camera model
  std::string model_type = CameraSettings::sneakParameterFromFile<std::string>("type", filepath);
  if (model_type == "pinhole")
    return load<PinholeSettings>(filepath);
  else
    throw(std::out_of_range("Error! Camera type '" + model_type + "' not recognized."));
}

template <typename T>
CameraSettings::Ptr CameraSettingsFactory::load(const std::string &filepath)
{
  // Read from settings file
  auto settings = std::make_shared<T>();
  settings->loadFromFile(filepath);
  return std::move(settings);
}