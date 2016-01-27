#pragma once

#include <ze/common/types.h>

namespace ze {

//! Settings for the IMU
//! Check the following references for more information about the IMU parameters:
//! https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
//! Default parameters are for ADIS16448 IMU.
class ImuCalibration
{
public:

  ImuCalibration() = default;
  ~ImuCalibration() = default;

  //! Gyro noise density (sigma). [rad/s*1/sqrt(Hz)]
  double gyro_noise_density = 0.00073088444;

  //! Accelerometer noise density (sigma). [m/s^2*1/sqrt(Hz)]
  double acc_noise_density = 0.01883649;

  //! Gyro bias random walk (sigma). [rad/s^2*1/sqrt(Hz)]
  double gyro_bias_random_walk_sigma = 0.00038765;

  //! Accelerometer bias random walk (sigma). [m/s^3*1/sqrt(Hz)]
  double acc_bias_random_walk_sigma = 0.012589254;

  //! Norm of the Gravitational acceleration. [m/s^2]
  double gravity_magnitude = 9.81007;

  //! Coriolis acceleration (earth rotation rate).
  Vector3 omega_coriolis = Vector3::Zero();

  //! Accelerometer saturation. [m/s^2]
  double saturation_accel_max = 150;

  //! Gyroscope saturation. [rad/s]
  double saturation_omega_max = 7.8;

  //! Expected IMU Rate [1/s]
  double imu_rate = 20;

  //! Camera-IMU delay: delay_imu_cam = cam_ts - cam_ts_delay
  double delay_imu_cam = 0.0;

  //! Maximum delay camera-imu
  double max_imu_delta_t = 0.01;

  //! Print camera calibration.
  void print(std::ostream& out, const std::string& s = "IMU Calibration: ") const;

};

} // namespace ze
