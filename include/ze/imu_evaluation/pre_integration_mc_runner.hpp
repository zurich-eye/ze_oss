#pragma once

#include <ze/imu_evaluation/pre_integration_runner.hpp>
#include <ze/common/statistics.h>
#include <ze/common/thread_pool.hpp>

namespace ze {

//! A class to run monte-carlo simulations of an imu pre-integration setup.
class PreIntegratorMonteCarlo
{
public:

  PreIntegratorMonteCarlo(PreIntegrationRunner::Ptr preintegraton_runner,
                          PreIntegratorFactory::Ptr pre_integrator_factory,
                          size_t threads = 1)
    : preintegraton_runner_(preintegraton_runner)
    , pre_integrator_factory_(pre_integrator_factory)
    , threads_(threads)
  {
  }

  //! Run a simulation with a given number of monte carlo runs.
  //! Specify the start and end-times of the trajectory to simulate. These
  //! must be compatible with the underlying scenario.
  void simulate(size_t num_rounds, FloatType start, FloatType end)
  {
    PreIntegrator::Ptr pre_int_actual = preintegrateActual(start, end);
    D_R_ref_ = pre_int_actual->D_R_i_k();
    R_ref_ = pre_int_actual->R_i_k();

    // Threaded run of the monte carlo simulations.
    ThreadPool pool(threads_);
    std::vector<std::future<PreIntegrator::Ptr>> results;

    // A vector of simulated results.
    for (size_t i = 0; i < num_rounds; ++i)
    {
      results.emplace_back(
        pool.enqueue([i, start, end, this] {
          VLOG(1) << "Monte-Carlo run #" << i;
           PreIntegrator::Ptr pre_int_mc = preintegrateCorrupted(start, end);

          return pre_int_mc;
      }));
    }

    // Extract the results of the simulations
    for (auto iter = results.begin(); iter != results.end(); ++iter)
    {
      auto value = iter->get();
      D_R_mc_.push_back(value->D_R_i_k());
      R_mc_.push_back(value->R_i_k());
    }

    // estimate the variance / covariance
    covariances_ = covariance_estimate(D_R_ref_, D_R_mc_);
    covariances_absolute_ = covariance_estimate(R_ref_, R_mc_);
  }

  //! Perform a pre-integration of the actual (exact, noise-free) trajectory.
  PreIntegrator::Ptr preintegrateActual(FloatType start, FloatType end)
  {
    // Create a new container to integrate and store the results.
    PreIntegrator::Ptr pre_integrator = pre_integrator_factory_->get();

    preintegraton_runner_->process(pre_integrator,
                                   false,
                                   start,
                                   end);

    return pre_integrator;
  }

  //! Perform a pre-integration of the corrupted (bias, noisy) trajectory.
  //! Every call to this function will generate different results due to
  //! the repeated noise generation in the scenarios. The bias will not change
  //! between calls.
  PreIntegrator::Ptr preintegrateCorrupted(FloatType start, FloatType end)
  {
    // Create a new container to integrate and store the results.
    PreIntegrator::Ptr pre_integrator = pre_integrator_factory_->get();

    preintegraton_runner_->process(pre_integrator,
                                   true,
                                   start,
                                   end);

    return pre_integrator;
  }

  const std::vector<Matrix3>& covariances()
  {
    return covariances_;
  }

  const std::vector<Matrix3>& covariances_absolute()
  {
    return covariances_absolute_;
  }

  const std::vector<std::vector<Matrix3>>& D_R_mc()
  {
    return D_R_mc_;
  }

private:
  PreIntegrationRunner::Ptr preintegraton_runner_;
  PreIntegratorFactory::Ptr pre_integrator_factory_;

  //! The number of threads to run the simulation on.
  size_t threads_;

  //! Vector of the results of the monte carlo runs (Absolute and Relative Orientations)
  std::vector<std::vector<Matrix3>> D_R_mc_;
  std::vector<std::vector<Matrix3>> R_mc_;

  //! Estiamted covariances for the relative motion and absolute motion.
  std::vector<Matrix3> covariances_;
  std::vector<Matrix3> covariances_absolute_;

  //! The reference values of the relative and absolute orientation.
  std::vector<Matrix3> D_R_ref_;
  std::vector<Matrix3> R_ref_;

  //! Estimate the covariance of the experiment set (relative orientation).
  std::vector<Matrix3> covariance_estimate(std::vector<Matrix3> D_R_actual,
                                           std::vector<std::vector<Matrix3>> D_R_mc)
  {
    CHECK_GT(D_R_mc.size(), 1u) << "Cannot estimate covariance for less than"
                                << "2 runs.";

    std::vector<Matrix3> covariances;

    //! Estimate the covariance matrix for every integration step.
    for (size_t step = 0; step < D_R_actual.size(); ++step)
    {
      Eigen::Matrix<FloatType, 3, Eigen::Dynamic> errors;
      errors.resize(3, D_R_mc.size());

      for (size_t run = 0; run < D_R_mc.size(); ++run)
      {
        Matrix3 delta = D_R_actual[step].transpose() * D_R_mc[run][step];
        Quaternion delta_q(delta);
        errors.col(run) = Quaternion::log(delta_q);
      }

      covariances.push_back(measurementCovariance(errors));
    }

    return covariances;
  }
};

} // namespace ze
