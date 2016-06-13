#pragma once

#include <ze/common/types.h>
#include <ze/imu_evaluation/scenario_runner.hpp>
#include <ze/imu_evaluation/manifold_pre_integrator.hpp>

namespace ze {

//! Pre-integrate a given scenario, at a given sampling rate using
//! a given pre-integration state.
class PreIntegrationRunner
{
public:
  ZE_POINTER_TYPEDEFS(PreIntegrationRunner);

  PreIntegrationRunner(ScenarioRunner::Ptr scenario_runner,
                       FloatType imu_sampling_time,
                       FloatType camera_sampling_time)
    : scenario_runner_(scenario_runner)
    , imu_sampling_time_(imu_sampling_time)
    , camera_sampling_time_(camera_sampling_time)
  {
    CHECK_LE(imu_sampling_time_, camera_sampling_time_)
        << "IMU Sampling time must be smaller than camera's";
  }

  //! Process the whole scenario given a start and end-time.
  void process(PreIntegrator::Ptr pre_integrator,
               bool corrupted,
               FloatType start,
               FloatType end)
  {
    FloatType t = start;
    FloatType next_camera_sample = start + camera_sampling_time_;
    std::vector<FloatType> times;
    std::vector<Vector6> imu_measurements;
    for (FloatType t = start; t <= end; t += imu_sampling_time_)
    {
      times.push_back(t);
      Vector6 measurement;
      if (corrupted)
      {
        measurement.head<3>() = scenario_runner_->specific_force_corrupted(t);
        measurement.tail<3>() = scenario_runner_->angular_velocity_corrupted(t);
      }
      else
      {
        measurement.head<3>() = scenario_runner_->specific_force_actual(t);
        measurement.tail<3>() = scenario_runner_->angular_velocity_actual(t);
      }
      imu_measurements.push_back(measurement);

      // Wait for the next camera sample and push the collected data to the
      // integrator.
      if (t > next_camera_sample)
      {
        next_camera_sample = t + camera_sampling_time_;
        pre_integrator->pushD_R_i_j(times, imu_measurements);
        times.clear();
        imu_measurements.clear();
      }
    }
  }

private:
  ScenarioRunner::Ptr scenario_runner_;

  //! The sampling time of the imu and camera. The camera's sampling interval
  //! should be larger than the imu's.
  FloatType imu_sampling_time_;
  FloatType camera_sampling_time_;
};

} // namespace ze
