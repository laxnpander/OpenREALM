

#ifndef PROJECT_DENSIFIER_SETTINGS_FACTORY_H
#define PROJECT_DENSIFIER_SETTINGS_FACTORY_H

#include <realm_densifier_base/densifier_settings.h>

namespace realm
{

class DensifierSettingsFactory
{
  public:
    /*!
     * @brief Loads densifier settings from a .yaml file. According to the parameter "type: FRAMEWORK" the correct
     *        settings are selected. Depending on the settings afterwards the correct densification framework is loaded.
     * @param filepath Absolute path of the settings .yaml file
     * @param directory Directory of the settings file. Is only necessary if the loaded framework needs additional
     *        files provided relative to the settings file
     * @return Densifier settings file
     */
    static DensifierSettings::Ptr load(const std::string &filepath, const std::string &directory);
  private:
    /*!
     * @brief Default loading function for standard settings. Can be specialized if more complex loading has to be performed.
     * @tparam T Type of the settings, see all derived classes from "DensifierSettings"
     * @param filepath Absolute path of the settings file
     * @param directory Directory of the settings file (optional)
     * @return Densifier settings according to the template provided
     */
    template<typename T>
    static DensifierSettings::Ptr loadDefault(const std::string &filepath, const std::string &directory);
};

} // namespace realm

#endif //PROJECT_DENSIFIER_SETTINGS_FACTORY_H
