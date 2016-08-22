#pragma once

#include <memory>
#include <ze/common/macros.h>
#include <ze/common/transformation.h>
#include <ze/vi_simulation/camera_simulator_types.hpp>

namespace ze {

// fwd
class CameraRig;
class CameraSimulator;
struct CameraSimulatorOptions;
class TrajectorySimulator;
class ImuSimulator;
class Visualizer;

// -----------------------------------------------------------------------------
struct ViSensorData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Nanosecond timestamp.
  int64_t timestamp;

  //! Vector of keypoint measurements.
  CameraMeasurementsVector cam_measurements;

  //! Vector of imu timestamps since last camera measurement.
  ImuStamps imu_stamps;

  //! Inertial measurements since last camera measurement.
  ImuAccGyrContainer imu_measurements;

  //! Groundtruth states.
  struct Groundtruth
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Transformation T_W_Bk;
    Vector3 acc_bias;
    Vector3 gyr_bias;
    Vector3 linear_velocity_W;
    Vector3 angular_velocity_B;

  };
  Groundtruth groundtruth;
};

// -----------------------------------------------------------------------------
class ViSimulator
{
public:
  ZE_POINTER_TYPEDEFS(ViSimulator);

  ViSimulator(
      const std::shared_ptr<TrajectorySimulator>& trajectory,
      const std::shared_ptr<CameraRig>& camera_rig,
      const CameraSimulatorOptions& camera_sim_options,
      const real_t gyr_bias_noise_sigma = 0.0000266,
      const real_t acc_bias_noise_sigma = 0.000433,
      const real_t gyr_noise_sigma = 0.000186,
      const real_t acc_noise_sigma = 0.00186,
      const uint32_t cam_bandwidth_hz = 20,
      const uint32_t imu_bandwidth_hz = 200,
      const real_t gravity_magnitude = 9.81);

  void initialize();

  std::pair<ViSensorData, bool> getMeasurement();

  void setVisualizer(const std::shared_ptr<Visualizer>& visualizer);

  void visualize(
      real_t dt = 0.2,
      real_t marker_size_trajectory = 0.2,
      real_t marker_size_landmarks = 0.2);

private:
  // Modules
  std::shared_ptr<TrajectorySimulator> trajectory_;
  std::shared_ptr<ImuSimulator> imu_;
  std::shared_ptr<CameraSimulator> camera_;
  std::shared_ptr<Visualizer> viz_;

  // Sampling state
  int64_t cam_dt_ns_;
  int64_t imu_dt_ns_;
  int64_t last_sample_stamp_ns_;
  Transformation T_W_Bk_;
};

// -----------------------------------------------------------------------------

//! Outdoor car scenario with stereo camera.
ViSimulator::Ptr createViSimulationScenario1();

} // namespace ze
