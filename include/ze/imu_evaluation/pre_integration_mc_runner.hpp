#pragma once

#include <ze/imu_evaluation/pre_integration_runner.hpp>

namespace ze {

//! A class to run monte-carlo simulations of an imu pre-integration setup.
template<class PRE_INTEGRATOR>
class PreIntegratorMonteCarlo
{
public:
  typedef PRE_INTEGRATOR pre_integrator_t;

  PreIntegratorMonteCarlo(PreIntegrationRunner::Ptr preintegraton_runner,
                          Matrix3 gyroscope_noise_covariance)
    : preintegraton_runner_(preintegraton_runner)
    , gyroscope_noise_covariance_(gyroscope_noise_covariance)
  {
  }

  //! Run a simulation with a given number of monte carlo runs.
  //! Specify the start and end-times of the trajectory to simulate. These
  //! must be compatible with the underlying scenario.
  void simulate(size_t num_rounds, FloatType start, FloatType end)
  {
    std::vector<Matrix3> D_R_actual = preintegrate_actual(start, end);

    // A vector of simulated results.
    std::vector<std::vector<Matrix3>> D_R_mc;
    for (size_t i = 0; i < num_rounds; ++i)
    {
      D_R_mc.push_back(preintegrate_corrupted(start, end));
    }

    // estimate the variance / covariance
    std::vector<Matrix3> covariance = covariance_estimate(D_R_actual, D_R_mc);
  }

  //! Perform a pre-integration of the actual (exact, noise-free) trajectory.
  std::vector<Matrix3> preintegrate_actual(FloatType start, FloatType end)
  {
    // Create a new container to integrate and store the results.
    typename pre_integrator_t::Ptr pre_integrator(
          std::make_shared<pre_integrator_t>(gyroscope_noise_covariance_));

    preintegraton_runner_->process(pre_integrator,
                                   false,
                                   start,
                                   end);

    return pre_integrator->getD_R();
  }

  //! Perform a pre-integration of the corrupted (bias, noisy) trajectory.
  //! Every call to this function will generate different results due to
  //! the repeated noise generation in the scenarios. The bias will not change
  //! between calls.
  std::vector<Matrix3> preintegrate_corrupted(FloatType start, FloatType end)
  {
    // Create a new container to integrate and store the results.
    typename pre_integrator_t::Ptr pre_integrator(
          std::make_shared<pre_integrator_t>(gyroscope_noise_covariance_));

    preintegraton_runner_->process(pre_integrator,
                                   false,
                                   start,
                                   end);

    return pre_integrator->getD_R();
  }

private:
  PreIntegrationRunner::Ptr preintegraton_runner_;
  Matrix3 gyroscope_noise_covariance_;

  //! Estimate the covariance of the experiment set
  std::vector<Matrix3> covariance_estimate(std::vector<Matrix3> D_R_actual,
                              std::vector<std::vector<Matrix3>> D_R_mc)
  {
    CHECK_GT(D_R_mc.size(), 1u) << "Cannot estimate covariance for less than"
                               << "2 runs.";

    std::vector<Matrix3> covariances;

    //! Estimate the covariance matrix for every integration step.
    for (size_t step = 0; step < D_R_actual.size(); ++step)
    {
      Eigen::Matrix<FloatType, Eigen::Dynamic, 3> errors;
      errors.resize(D_R_mc.size(), 3);

      for (size_t run = 0; run < D_R_mc.size(); ++run)
      {
        Matrix3 delta = D_R_actual[step].transpose() * D_R_mc[run][step];
        Quaternion delta_q(delta);
        errors.row(run) = Quaternion::log(delta_q).transpose();
      }
      Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic> zero_mean
          = errors.rowwise() - errors.rowwise().mean().transpose();

      covariances.push_back((zero_mean.adjoint() * zero_mean)
                            / (zero_mean.rows() - 1));
    }

    return covariances;
  }
};

} // namespace ze
