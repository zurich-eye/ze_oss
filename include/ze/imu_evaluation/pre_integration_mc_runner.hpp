#pragma once

namespace ze {

//! A class to run monte-carlo simulations of an imu pre-integration setup.
template<PRE_INTEGRATOR>
class PreIntegratorMonteCarlo
{
public:
  typedef typename PRE_INTEGRATOR pre_integrator_t;

  PreIntegratorMonteCarlo(PreIntegrationRunner::Ptr preintegraton_runner)
    : preintegraton_runner_(preintegraton_runner)
  {
  }

  //! Run a simulation with a given number of monte carlo runs.
  void simulate(size_t num_rounds)
  {
    std::vector<Matrix3> D_R_actual = preintegrate_actual();

    // A vector of simulated results.
    std::vector<std::vector<Matrix3>> D_R_mc;
    for (size_t i = 0; i < num_rounds; ++i)
    {
      D_R_mc.push_back(preintegrate_corrupted());
    }

    // estimate the variance / covariance
    Matrix3 covariance_estimate(D_R_actual, D_R_mc);
  }

  //! Perform a pre-integration of the actual (exact, noise-free) trajectory.
  std::vector<Matrix3> preintegrate_actual()
  {
    // Create a new container to integrate and store the results.
    pre_integrator_t::Ptr pre_integrator(std::make_shared<pre_integrator_t>());

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
  std::vector<Matrix3> preintegrate_corrupted()
  {
    // Create a new container to integrate and store the results.
    pre_integrator_t::Ptr pre_integrator(std::make_shared<pre_integrator_t>());

    preintegraton_runner_->process(pre_integrator,
                                   false,
                                   start,
                                   end);

    return pre_integrator->getD_R();
  }

private:
  PreIntegrationRunner::Ptr preintegraton_runner_;

  //! Estimate the covariance of the experiment set
  std::vector<Matrix3> covariance_estimate(std::vector<Matrix3> D_R_actual,
                              std::vector<std::vector<Matrix3>> D_R_mc)
  {
    CHECK_GT(D_R_mc.size(), 1) << "Cannot estimate covariance for less than"
                               << "2 runs.";

    std::vector<Matrix3> covariances;

    //! Estimate the covariance matrix for every integration step.
    for (size_t step = 0; step < D_R_actual.size(); ++step)
    {
      Eigen::Matrix<FloatType, Eigen::Dynamic, 3> errors;
      errors.resize(D_R_mc.size(), 3);

      for (size_t run = 0; run < D_R_mc.size(); ++run)
      {
        errors.row(run) = Log(D_R_actual[step].transpose() * D_R_mc[run][step]);
      }
      Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic> zero_mean
          = errors.rowWise() - errors.rowWise().mean();

      covariances.push_back((zero_mean.adjoint() * zero_mean)
                            / (zero_mean.rows() - 1));
    }

    return covariances;
  }
};

} // namespace ze
