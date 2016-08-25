// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/types.hpp>
#include <ze/data_provider/data_provider_base.hpp>
#include <ze/imu_evaluation/pre_integrator_base.hpp>
#include <ze/vi_simulation/imu_simulator.hpp>

namespace ze {

//! Pre-integrate a given scenario, at a given sampling rate using
//! a given pre-integration state.
class PreIntegrationRunner
{
public:
  ZE_POINTER_TYPEDEFS(PreIntegrationRunner);

  //! Negative sampling values for camera sampling implies a single run where
  //! all imu samples are processed in a single step.
  PreIntegrationRunner(ImuSimulator::Ptr scenario_runner,
                       real_t imu_sampling_time,
                       real_t camera_sampling_time = -1);

  //! Optionally set the initial value for the absolute orientation integrator.
  void setInitialOrientation(Matrix3 initial_orientation);

  //! Process the whole scenario given a start and end-time.
  void process(PreIntegrator::Ptr pre_integrator,
               bool corrupted,
               real_t start,
               real_t end);

private:
  ImuSimulator::Ptr scenario_runner_;

  //! The sampling time of the imu and camera. The camera's sampling interval
  //! should be larger than the imu's.
  real_t imu_sampling_time_;
  real_t camera_sampling_time_;

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
  std::vector<real_t> times_;
  ImuAccGyrContainer imu_measurements_;

  //! Load the bag data into memory.
  void loadData();
};

} // namespace ze
