#include <ze/imu_evaluation/pre_integration_mc_runner.hpp>

namespace ze {

//------------------------------------------------------------------------------
PreIntegratorMonteCarlo::PreIntegratorMonteCarlo(
    PreIntegrationRunner::Ptr preintegraton_runner,
    PreIntegratorFactory::Ptr pre_integrator_factory,
    size_t threads)
  : preintegration_runner_(preintegraton_runner)
  , pre_integrator_factory_(pre_integrator_factory)
  , threads_(threads)
{
}

//------------------------------------------------------------------------------
void PreIntegratorMonteCarlo::simulate(size_t num_rounds,
                                       real_t start,
                                       real_t end)
{
  PreIntegrator::Ptr pre_int_actual = preintegrateActual(start, end);
  D_R_ref_ = pre_int_actual->D_R_i_k();

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
  }

  // estimate the variance / covariance
  covariances_ = covarianceEstimates(D_R_ref_, D_R_mc_);
}

//------------------------------------------------------------------------------
PreIntegrator::Ptr PreIntegratorMonteCarlo::preintegrateActual(
    real_t start, real_t end)
{
  // Create a new container to integrate and store the results.
  PreIntegrator::Ptr pre_integrator = pre_integrator_factory_->get();

  preintegration_runner_->process(pre_integrator,
                                 false,
                                 start,
                                 end);

  return pre_integrator;
}

//------------------------------------------------------------------------------
PreIntegrator::Ptr PreIntegratorMonteCarlo::preintegrateCorrupted(
    real_t start, real_t end)
{
  // Create a new container to integrate and store the results.
  PreIntegrator::Ptr pre_integrator = pre_integrator_factory_->get();

  preintegration_runner_->process(pre_integrator,
                                 true,
                                 start,
                                 end);

  return pre_integrator;
}

//------------------------------------------------------------------------------
std::vector<Matrix3> PreIntegratorMonteCarlo::covarianceEstimates(
    std::vector<Matrix3> D_R_actual,
    std::vector<std::vector<Matrix3>> D_R_mc)
{
  CHECK_GT(D_R_mc.size(), 1u) << "Cannot estimate covariance for less than"
                              << "2 runs.";

  std::vector<Matrix3> covariances;

  //! Estimate the covariance matrix for every integration step.
  for (size_t step = 0; step < D_R_actual.size(); ++step)
  {
    Eigen::Matrix<real_t, 3, Eigen::Dynamic> errors;
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

//------------------------------------------------------------------------------
void PreIntegratorMonteCarlo::clean()
{
  D_R_mc_ = std::vector<std::vector<Matrix3>>();
  D_R_ref_ = std::vector<Matrix3>();
}

} // namespace ze
