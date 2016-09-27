// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <memory>
#include <ze/common/macros.hpp>
#include <ze/common/transformation.hpp>
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
      const uint32_t cam_framerate_hz = 20,
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
