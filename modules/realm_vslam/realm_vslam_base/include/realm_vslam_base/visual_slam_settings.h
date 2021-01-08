

#ifndef PROJECT_VISUAL_SLAM_SETTINGS_H
#define PROJECT_VISUAL_SLAM_SETTINGS_H

#include <realm_core/settings_base.h>

namespace realm
{

class VisualSlamSettings : public SettingsBase
{
  public:
    using Ptr = std::shared_ptr<VisualSlamSettings>;
    using ConstPtr = std::shared_ptr<const VisualSlamSettings>;
  public:
    VisualSlamSettings()
    {
      add("type", Parameter_t<std::string>{"", "Framework for visual SLAM."});
      add("resizing", Parameter_t<double>{0.0, "Resize factor of input images."});
    }
};

class OrbSlamSettings : public VisualSlamSettings
{
  public:
    OrbSlamSettings()
    {
      add("nrof_features", Parameter_t<int>{0, "ORB Extractor: Number of features per image."});
      add("scale_factor", Parameter_t<double>{0, "ORB Extractor: Scale factor between levels in the scale pyramid."});
      add("n_pyr_levels", Parameter_t<int>{0, "ORB Extractor: Number of levels in the scale pyramid."});
      add("ini_th_FAST", Parameter_t<int>{0, "Initial FAST features threshold for detection."});
      add("min_th_FAST", Parameter_t<int>{0, "Minimum response of FAST features."});
      add("path_vocabulary", Parameter_t<std::string>{"", "Path to ORB_SLAM2 vocabulary file."});
    }
};

class SvoSettings : public VisualSlamSettings
{
  public:
    SvoSettings()
    {
    }
};

class Svo2Settings : public VisualSlamSettings
{
  public:
    Svo2Settings()
    {
    }
};

class DsoSettings : public VisualSlamSettings
{
  public:
    DsoSettings()
    {
      // TODO: Add help
      add("desiredImmatureDensity", Parameter_t<double>{0, ""});
      add("desiredPointDensity", Parameter_t<double>{0, ""});
      add("minFrames", Parameter_t<int>{0, ""});
      add("maxFrames", Parameter_t<int>{0, ""});
      add("maxOptIterations", Parameter_t<int>{0, ""});
      add("minOptIterations", Parameter_t<int>{0, ""});
      add("logStuff", Parameter_t<int>{0, ""});
      add("kfGlobalWeight", Parameter_t<double>{0, ""});
      add("photometricCalibration", Parameter_t<int>{0, ""});
      add("affineOptModeA", Parameter_t<double>{0, ""});
      add("affineOptModeB", Parameter_t<double>{0, ""});
    }
};

class OpenVslamSettings : public VisualSlamSettings
{
public:
  OpenVslamSettings()
  {
    add("nrof_features", Parameter_t<int>{0, "ORB Extractor: Number of features per image."});
    add("scale_factor", Parameter_t<double>{0, "ORB Extractor: Scale factor between levels in the scale pyramid."});
    add("n_pyr_levels", Parameter_t<int>{0, "ORB Extractor: Number of levels in the scale pyramid."});
    add("ini_th_FAST", Parameter_t<int>{0, "Initial FAST features threshold for detection."});
    add("min_th_FAST", Parameter_t<int>{0, "Minimum response of FAST features."});
    add("path_vocabulary", Parameter_t<std::string>{"", "Path to ORB_SLAM2 vocabulary file."});
  }
};

} // namespace realm

#endif //PROJECT_VISUAL_SLAM_SETTINGS_H
