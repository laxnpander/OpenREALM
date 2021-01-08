

#include <realm_stages/stage_settings_factory.h>
#include <realm_stages/stage_settings.h>

using namespace realm;

StageSettings::Ptr StageSettingsFactory::load(const std::string &stage_type_set,
                                              const std::string &filepath)
{
  std::string stage_type_read = StageSettings::sneakParameterFromFile<std::string>("type", filepath);
  if (stage_type_set != stage_type_read)
    throw(std::invalid_argument("Error: Could not load stage settings. Stage type mismatch!"));

  // Desired stage type and settings file match, start loading file
  if (stage_type_set == "pose_estimation")
    return loadDefault<PoseEstimationSettings>(stage_type_set, filepath);
  if (stage_type_set == "densification")
    return loadDefault<DensificationSettings>(stage_type_set, filepath);
  if (stage_type_set == "surface_generation")
    return loadDefault<SurfaceGenerationSettings>(stage_type_set, filepath);
  if (stage_type_set == "ortho_rectification")
    return loadDefault<OrthoRectificationSettings>(stage_type_set, filepath);
  if (stage_type_set == "mosaicing")
    return loadDefault<MosaicingSettings>(stage_type_set, filepath);
  if (stage_type_set == "tileing")
    return loadDefault<TileingSettings>(stage_type_set, filepath);
  throw(std::invalid_argument("Error: Could not load stage settings. Did not recognize stage type: " + stage_type_set));
}

template <typename T>
StageSettings::Ptr StageSettingsFactory::loadDefault(const std::string &stage_type_set, const std::string &filepath)
{
  auto settings = std::make_shared<T>();
  settings->loadFromFile(filepath);
  return std::move(settings);
}