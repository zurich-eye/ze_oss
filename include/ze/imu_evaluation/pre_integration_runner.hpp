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

  //! Negative sampling values for camera sampling implies a single run where
  //! all imu samples are processed in a single step.
  PreIntegrationRunner(ScenarioRunner::Ptr scenario_runner,
                       FloatType imu_sampling_time,
                       FloatType camera_sampling_time = -1)
    : scenario_runner_(scenario_runner)
    , imu_sampling_time_(imu_sampling_time)
    , camera_sampling_time_(camera_sampling_time)
    , initial_orientation_(Matrix3::Identity())
  {
    if (camera_sampling_time_ >= 0)
    {
      CHECK_LE(imu_sampling_time_, camera_sampling_time_)
          << "IMU Sampling time must be smaller than camera's";
    }
  }

  //! Optionally set the initial value for the absolute orientation integrator.
  void setInitialOrientation(Matrix3 initial_orientation)
  {
    initial_orientation_ = initial_orientation;
  }

  //! Process the whole scenario given a start and end-time.
  void process(PreIntegrator::Ptr pre_integrator,
               bool corrupted,
               FloatType start,
               FloatType end)
  {
    FloatType t = start;
    pre_integrator->setInitialOrientation(initial_orientation_);

    // For negative camera sampling rates, a single batch of imu samples between
    // start and end is taken.
    FloatType next_camera_sample;
    int samples_per_batch;

    std::vector<FloatType> times;
    // Worst case container size allocation.
    if (camera_sampling_time_ < 0)
    {
      next_camera_sample = end;
      samples_per_batch =static_cast<int>(
                           std::ceil(end - start) / imu_sampling_time_) + 1;
    }
    else
    {
      next_camera_sample = start + camera_sampling_time_;
      samples_per_batch = static_cast<int>(
                            std::ceil(camera_sampling_time_ / imu_sampling_time_)) + 1;
    }

    ImuAccGyr imu_measurements(6, samples_per_batch);

    int i = 0;
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
      imu_measurements.col(i) = measurement;

      // Wait for the next camera sample and push the collected data to the
      // integrator.
      if (t > next_camera_sample)
      {
        next_camera_sample = t + camera_sampling_time_;
        pre_integrator->pushD_R_i_j(times, imu_measurements.leftCols(times.size()));
        times.clear();
        i = 0;
      }

      ++i;
    }

    // Ensure that all results are pushed to the container.
    if (imu_measurements.size() != 0)
    {
      pre_integrator->pushD_R_i_j(times, imu_measurements.leftCols(times.size()));
    }
  }

private:
  ScenarioRunner::Ptr scenario_runner_;

  //! The sampling time of the imu and camera. The camera's sampling interval
  //! should be larger than the imu's.
  FloatType imu_sampling_time_;
  FloatType camera_sampling_time_;

  //! An initial value for the orientation
  Matrix3 initial_orientation_;
};

} // namespace ze
