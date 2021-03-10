
#ifndef OPENREALM_IMU_SETTINGS_H
#define OPENREALM_IMU_SETTINGS_H

#include <realm_core/settings_base.h>

namespace realm {

class ImuSettings : public SettingsBase
{
public:
  using Ptr = std::shared_ptr<ImuSettings>;
  using ConstPtr = std::shared_ptr<const ImuSettings>;
public:
  ImuSettings()
  {
    add("type", Parameter_t<std::string>{"", "IMU model"});
    add("gyro_noise_density", Parameter_t<double>{0.0, "Gyroscope noise density"});
    add("gyro_bias_random_walk_noise_density", Parameter_t<double>{0.0, "Gyroscope bias"});
    add("acc_noise_density", Parameter_t<double>{0.0, "Accelerometer noise denisty"});
    add("acc_bias_random_walk_noise_density", Parameter_t<double>{0.0, "Accelerometer bias"});
    add("T_cam_imu", Parameter_t<cv::Mat>{cv::Mat(), "Camera to body transformation matrix"});
    add("freq", Parameter_t<double>{0.0, "Frequency of the IMU"});
  }
};

}

#endif //OPENREALM_IMU_SETTINGS_H
