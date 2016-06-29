#pragma once

#include <ze/common/types.h>
#include <ze/data_provider/data_provider_base.hpp>
#include <ze/imu_evaluation/pre_integrator_base.hpp>
#include <ze/imu_evaluation/scenario_runner.hpp>

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
                       FloatType camera_sampling_time = -1);

  //! Optionally set the initial value for the absolute orientation integrator.
  void setInitialOrientation(Matrix3 initial_orientation);

  //! Process the whole scenario given a start and end-time.
  void process(PreIntegrator::Ptr pre_integrator,
               bool corrupted,
               FloatType start,
               FloatType end);

private:
  ScenarioRunner::Ptr scenario_runner_;

  //! The sampling time of the imu and camera. The camera's sampling interval
  //! should be larger than the imu's.
  FloatType imu_sampling_time_;
  FloatType camera_sampling_time_;

  //! An initial value for the orientation
  Matrix3 initial_orientation_;
};


//! Pre-integrate given a data provider source of imu data.
//! The integrator that runs on real data has the following restrictions:
//!   _ only allows to integrate a full dataset
//!   _ the dataprovider should only listen for a single imu topic, and exactly
//!     one imu topic.
class PreIntegrationRunnerDataProvider
{
public:
  ZE_POINTER_TYPEDEFS(PreIntegrationRunnerDataProvider);

  //! Negative sampling values for camera sampling implies a single run where
  //! all imu samples are processed in a single step.
  PreIntegrationRunnerDataProvider(
      DataProviderBase::Ptr data_provider);

  //! Optionally set the initial value for the absolute orientation integrator.
  void setInitialOrientation(Matrix3 initial_orientation);

  //! Process the whole dataset in a single run.
  void process(PreIntegrator::Ptr pre_integrator);

private:
  DataProviderBase::Ptr data_provider_;

  //! An initial value for the orientation
  Matrix3 initial_orientation_;

  //! Cached values for times and measurements
  std::vector<FloatType> times_;
  ImuAccGyrContainer imu_measurements_;

  //! Load the bag data into memory.
  void loadData();
};

} // namespace ze
