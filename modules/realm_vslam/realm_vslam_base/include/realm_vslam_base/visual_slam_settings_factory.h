

#ifndef PROJECT_VISUAL_SLAM_SETTINGS_FACTORY_H
#define PROJECT_VISUAL_SLAM_SETTINGS_FACTORY_H

#include <realm_vslam_base/visual_slam_settings.h>

namespace realm
{

class VisualSlamSettingsFactory
{
  public:
    /*!
     * @brief Function to create visual slam settings from .yaml file
     * @param filepath Absolute path to the settings yaml file
     * @param directory Absolute path to the settings directory (used for possible additional files needed to be included)
     * @return Loaded visual slam settings
     */
    static VisualSlamSettings::Ptr load(const std::string &filepath,
                                        const std::string &directory);
  private:
    static VisualSlamSettings::Ptr loadOrbSlam2(const std::string &filepath, const std::string &directory);
    static VisualSlamSettings::Ptr loadOpenVslam(const std::string &filepath, const std::string &directory);
    template <typename T>
    static VisualSlamSettings::Ptr loadDefault(const std::string &filepath, const std::string &directory);
};

} // namespace realm

#endif //PROJECT_VISUAL_SLAM_SETTINGS_FACTORY_H
