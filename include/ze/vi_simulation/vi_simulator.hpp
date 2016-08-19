#pragma once

#include <ze/common/types.h>
#include <ze/vi_simulation/trajectory_simulator.hpp>
#include <ze/vi_simulation/camera_simulator.hpp>
#include <ze/vi_simulation/imu_simulator.hpp>

namespace ze {

struct ViSensorData
{
  int64_t cam_timestamp_;
  CameraMeasurementsVector cam_measurements_;
  ImuStamps imu_stamps_;
  ImuAccGyrContainer imu_measurements_;
};

class ViSimulator
{
public:
  ZE_POINTER_TYPEDEFS(ViSimulator);

  ViSimulator(
      const TrajectorySimulator::Ptr& trajectory,
      const CameraRig::Ptr& camera_rig,
      const CameraSimulatorOptions& camera_sim_options,
      const FloatType gyr_bias_noise_sigma = 0.0000266,
      const FloatType acc_bias_noise_sigma = 0.000433,
      const FloatType gyr_noise_sigma = 0.000186,
      const FloatType acc_noise_sigma = 0.00186,
      const uint32_t cam_bandwidth_hz = 20,
      const uint32_t imu_bandwidth_hz = 200,
      const FloatType gravity_magnitude = 9.81);

  std::pair<ViSensorData, bool> getMeasurement();

private:
  // Modules
  TrajectorySimulator::Ptr trajectory_;
  ImuSimulator::Ptr imu_;
  CameraSimulator::Ptr camera_;

  // Sampling state
  int64_t cam_dt_ns_;
  int64_t imu_dt_ns_;
  int64_t last_sample_stamp_;
};

} // namespace ze
