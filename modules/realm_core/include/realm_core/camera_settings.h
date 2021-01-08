

#ifndef PROJECT_CAMERA_SETTINGS_H
#define PROJECT_CAMERA_SETTINGS_H

#include <string>
#include <memory>

#include <realm_core/structs.h>
#include <realm_core/settings_base.h>

namespace realm
{

/*!
 * @brief Simple class for camera settings, inheriting from the settings base class. See also settings_base.h for more
 *        informations about the general structure. Note, that the add function is not public, therefore parameters can
 *        only be added inside the class.
 */
class CameraSettings : public SettingsBase
{
  public:
    using Ptr = std::shared_ptr<CameraSettings>;
    using ConstPtr = std::shared_ptr<const CameraSettings>;
  public:
    CameraSettings()
    {
      add("type", Parameter_t<std::string>{"", "Model of the camera, e.g. pinhole or fisheye"});
      add("fps", Parameter_t<double>{0, "Frames per second"});
      add("width", Parameter_t<int>{0, "Width of the image"});
      add("height", Parameter_t<int>{0, "Height of the image"});
    }
};

class PinholeSettings : public CameraSettings
{
  public:
    PinholeSettings() : CameraSettings()
    {
      add("cx", Parameter_t<double>{0.0, "Principal point in x-dir"});
      add("cy", Parameter_t<double>{0.0, "Principal point in y-dir"});
      add("fx", Parameter_t<double>{0.0, "Focal length in x-dir"});
      add("fy", Parameter_t<double>{0.0, "Focal length in y-dir"});
      add("k1", Parameter_t<double>{0.0, "Distortion coefficient k1"});
      add("k2", Parameter_t<double>{0.0, "Distortion coefficient k2"});
      add("p1", Parameter_t<double>{0.0, "Distortion coefficient p1"});
      add("p2", Parameter_t<double>{0.0, "Distortion coefficient p2"});
      add("k3", Parameter_t<double>{0.0, "Distortion coefficient k3"});
    }
};

} // namespace realm

#endif //PROJECT_CAMERA_SETTINGS_H
