#pragma once

#include <ze/imu_evaluation/pre_integration_runner.hpp>
#include <ze/common/statistics.h>
#include <ze/common/thread_pool.hpp>

namespace ze {

//! A class to run monte-carlo simulations of an imu pre-integration setup.
class PreIntegratorMonteCarlo
{
public:
  ZE_POINTER_TYPEDEFS(PreIntegratorMonteCarlo);

  PreIntegratorMonteCarlo(PreIntegrationRunner::Ptr preintegraton_runner,
                          PreIntegratorFactory::Ptr pre_integrator_factory,
                          size_t threads = 1);

  //! Run a simulation with a given number of monte carlo runs.
  //! Specify the start and end-times of the trajectory to simulate. These
  //! must be compatible with the underlying scenario.
  void simulate(size_t num_rounds, FloatType start, FloatType end);

  //! Perform a pre-integration of the actual (exact, noise-free) trajectory.
  PreIntegrator::Ptr preintegrateActual(FloatType start, FloatType end);

  //! Perform a pre-integration of the corrupted (bias, noisy) trajectory.
  //! Every call to this function will generate different results due to
  //! the repeated noise generation in the scenarios. The bias will not change
  //! between calls.
  PreIntegrator::Ptr preintegrateCorrupted(FloatType start, FloatType end);

  const std::vector<Matrix3>& covariances()
  {
    return covariances_;
  }

  const std::vector<std::vector<Matrix3>>& D_R_mc()
  {
    return D_R_mc_;
  }

  //! Clean the runner from unused temporaries to reduce the memory footprint.
  void clean();

private:
  PreIntegrationRunner::Ptr preintegration_runner_;
  PreIntegratorFactory::Ptr pre_integrator_factory_;

  //! The number of threads to run the simulation on.
  size_t threads_;

  //! Vector of the results of the monte carlo runs (Absolute and Relative Orientations)
  std::vector<std::vector<Matrix3>> D_R_mc_;

  //! Estiamted covariances for the relative motion and absolute motion.
  std::vector<Matrix3> covariances_;

  //! The reference values of the relative and absolute orientation.
  std::vector<Matrix3> D_R_ref_;
  std::vector<Matrix3> R_ref_;

  //! Estimate the covariance of the experiment set (relative orientation).
  std::vector<Matrix3> covarianceEstimates(
      std::vector<Matrix3> D_R_actual,
      std::vector<std::vector<Matrix3>> D_R_mc);

};

} // namespace ze
