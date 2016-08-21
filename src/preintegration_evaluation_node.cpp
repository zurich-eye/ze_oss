#include <functional>
#include <memory>
#include <initializer_list>

#include <gflags/gflags.h>
#include <ze/common/logging.hpp>

#include <imp/core/image.hpp>
#include <ze/common/file_utils.h>
#include <ze/common/timer.h>
#include <ze/common/transformation.h>
#include <ze/common/types.h>
#include <ze/data_provider/data_provider_factory.hpp>

#include <ze/imu_evaluation/pre_integration_runner.hpp>
#include <ze/imu_evaluation/pre_integration_mc_runner.hpp>
#include <ze/vi_simulation/evaluation_tools.hpp>
#include <ze/vi_simulation/imu_bias_simulator.hpp>
#include <ze/vi_simulation/imu_simulator.hpp>
#include <ze/vi_simulation/trajectory_simulator.hpp>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/common/csv_trajectory.h>
#include <ze/splines/viz_splines.hpp>
#include <ze/visualization/viz_ros.h>
#include <ze/matplotlib/matplotlibcpp.hpp>
#include <ze/splines/rotation_vector.hpp>
#include <ze/imu_evaluation/manifold_pre_integrator.hpp>
#include <ze/imu_evaluation/quaternion_pre_integrator.hpp>
#include <ze/imu_evaluation/imu_preintegration_parameters.hpp>
#include <ze/imu_evaluation/preintegration_evaluation_node.hpp>

DEFINE_bool(time_integration, false, "Show timings for the integrations?");
DEFINE_string(run_type, "drift", "'drift'' evaluation, 'covariance'' monte carlo run, 'real'' dataset run");
DEFINE_string(reference_trajectory_csv, "", "The path to the csv file that contains the reference trajectory for a real run (Euroc-format)");

namespace ze {

// -----------------------------------------------------------------------------
PreIntegrationEvaluationNode::PreIntegrationEvaluationNode()
{
  plt::ion();

  visualizer_ = std::make_shared<VisualizerRos>();
  splines_visualizer_ = std::make_shared<SplinesVisualizer>(visualizer_);

  parameters_ = ImuPreIntegrationParameters::fromGFlags();
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::runCovarianceMonteCarloMain()
{
  Vector3 accel_covar = parameters_.accel_noise_density
                        * parameters_.accel_noise_density
                        * Vector3::Ones();
  Vector3 gyro_covar = parameters_.gyro_noise_density
                       * parameters_.gyro_noise_density
                       * Vector3::Ones();
  Matrix3 gyroscope_noise_covariance = gyro_covar.asDiagonal();

  VLOG(1) << "Initialize noise models with \n"
          << " Accelerometer noise density: " << parameters_.accel_noise_density
          << " Gyroscope noise density: " << parameters_.gyro_noise_density;
  RandomVectorSampler<3>::Ptr accel_noise =
      RandomVectorSampler<3>::variances(accel_covar);
  RandomVectorSampler<3>::Ptr gyro_noise =
      RandomVectorSampler<3>::variances(gyro_covar);

  PreIntegrationRunner::Ptr preintegration_runner = getPreIntegrationRunner(
                                                      accel_noise,
                                                      gyro_noise);

  real_t start = trajectory_->t_min();
  real_t end = trajectory_->t_max() - 5;

  //! A single monte carlo run
  PreIntegratorMonteCarlo::Ptr mc = runManifoldCorruptedMc(
                                      preintegration_runner,
                                      PreIntegrator::FirstOrderMidward,
                                      "Manifold MWD",
                                      gyroscope_noise_covariance,
                                      start,
                                      end);

  //! Run the pre-integrators:

  // 1) Manifold Integration Forward
  ManifoldPreIntegrationState::Ptr state_corrupted_fwd =
      runManifoldCorrupted(preintegration_runner,
                           PreIntegrator::FirstOrderForward,
                           "Manifold Corr FWD",
                           gyroscope_noise_covariance,
                           start,
                           end,
                           mc);

  // 2) Manifold Integration Midward
  ManifoldPreIntegrationState::Ptr state_corrupted_mid =
      runManifoldCorrupted(preintegration_runner,
                           PreIntegrator::FirstOrderMidward,
                           "Manifold Corr MWD",
                           gyroscope_noise_covariance,
                           start,
                           end,
                           mc);

  // 4) Quaternion Integration: Forward Integration
  QuaternionPreIntegrationState::Ptr state_quat_fwd =
      runQuaternion(preintegration_runner,
                    PreIntegrator::FirstOrderForward,
                    "Quat FWD",
                    gyroscope_noise_covariance,
                    start,
                    end,
                    mc);

  // 5) Quaternion Integration: Forward Integration
  QuaternionPreIntegrationState::Ptr state_quat_mid =
      runQuaternion(preintegration_runner,
                    PreIntegrator::FirstOrderMidward,
                    "Quat MWD",
                    gyroscope_noise_covariance,
                    start,
                    end,
                    mc);

  // 6) Quaternion Integration: RK3
  QuaternionPreIntegrationState::Ptr state_quat_rk3 =
      runQuaternion(preintegration_runner,
                    PreIntegrator::RungeKutta3,
                    "RK3",
                    gyroscope_noise_covariance,
                    start,
                    end,
                    mc);

  // 7) Quaternion Integration: RK4
  QuaternionPreIntegrationState::Ptr state_quat_rk4 =
      runQuaternion(preintegration_runner,
                    PreIntegrator::RungeKutta4,
                    "RK4",
                    gyroscope_noise_covariance,
                    start,
                    end,
                    mc);

  // 8) Quaternion Integration: CG3
  QuaternionPreIntegrationState::Ptr state_quat_cg3 =
      runQuaternion(preintegration_runner,
                    PreIntegrator::CrouchGrossman3,
                    "CG3",
                    gyroscope_noise_covariance,
                    start,
                    end,
                    mc);

  // 9) Quaternion Integration: CG4
  QuaternionPreIntegrationState::Ptr state_quat_cg4 =
      runQuaternion(preintegration_runner,
                    PreIntegrator::CrouchGrossman4,
                    "CG4",
                    gyroscope_noise_covariance,
                    start,
                    end,
                    mc);
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::runDriftEvaluationMain()
{
  Vector3 accel_covar = parameters_.accel_noise_density
                        * parameters_.accel_noise_density
                        * Vector3::Ones();
  Vector3 gyro_covar = parameters_.gyro_noise_density
                       * parameters_.gyro_noise_density
                       * Vector3::Ones();
  Matrix3 gyroscope_noise_covariance = gyro_covar.asDiagonal();

  VLOG(1) << "Initialize noise models with \n"
          << " Accelerometer noise density: " << parameters_.accel_noise_density
          << " Gyroscope noise density: " << parameters_.gyro_noise_density;
  RandomVectorSampler<3>::Ptr accel_noise =
      RandomVectorSampler<3>::variances(accel_covar);
  RandomVectorSampler<3>::Ptr gyro_noise =
      RandomVectorSampler<3>::variances(gyro_covar);

  PreIntegrationRunner::Ptr preintegration_runner = getPreIntegrationRunner(
                                                      accel_noise,
                                                      gyro_noise);

  real_t start = trajectory_->t_min();
  real_t end = trajectory_->t_max() - 5;

  /////// Evaluate drifts:
 runDriftEvaluationRuns(
       200, gyroscope_noise_covariance, accel_noise, gyro_noise);
}



// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::runRealDatasetMain()
{
  plt::ion();

  visualizer_ = std::make_shared<VisualizerRos>();
  splines_visualizer_ = std::make_shared<SplinesVisualizer>(visualizer_);

  parameters_ = ImuPreIntegrationParameters::fromGFlags();

  Vector3 accel_covar = parameters_.accel_noise_density
                        * parameters_.accel_noise_density
                        * Vector3::Ones();
  Vector3 gyro_covar = parameters_.gyro_noise_density
                       * parameters_.gyro_noise_density
                       * Vector3::Ones();
  Matrix3 gyroscope_noise_covariance = gyro_covar.asDiagonal();

  VLOG(1) << "Initialize noise models with \n"
          << " Accelerometer noise density: " << parameters_.accel_noise_density
          << " Gyroscope noise density: " << parameters_.gyro_noise_density;
  RandomVectorSampler<3>::Ptr accel_noise =
      RandomVectorSampler<3>::variances(accel_covar);
  RandomVectorSampler<3>::Ptr gyro_noise =
      RandomVectorSampler<3>::variances(gyro_covar);

  DataProviderBase::Ptr data_provider = loadDataProviderFromGflags(1);

  // Load reference orientations from ground truth csv file
  EurocResultSeries reference_result_series;
  reference_result_series.load(FLAGS_reference_trajectory_csv);
  StampedTransformationVector T_ref_list =
      reference_result_series.getStampedTransformationVector();

  PreIntegrationRunnerDataProvider::Ptr preintegration_runner(
        std::make_shared<PreIntegrationRunnerDataProvider>(data_provider));
  preintegration_runner->setInitialOrientation(
        T_ref_list.front().second.getRotationMatrix());

  // Manifold FWD
  PreIntegrator::Ptr pi_manifold_fwd =
      std::make_shared<ManifoldPreIntegrationState>(
        gyroscope_noise_covariance,
        PreIntegrator::FirstOrderForward);
  pi_manifold_fwd->computeAbsolutes(true);
  preintegration_runner->process(pi_manifold_fwd);

  plotOrientation(pi_manifold_fwd->times_raw(),
                  pi_manifold_fwd->R_i_k(),
                  "Manifold FWD",
                  trajectory_);

  // Manifold MWD
  PreIntegrator::Ptr pi_manifold_mwd =
      std::make_shared<ManifoldPreIntegrationState>(
        gyroscope_noise_covariance,
        PreIntegrator::FirstOrderMidward);
  pi_manifold_mwd->computeAbsolutes(true);
  preintegration_runner->process(pi_manifold_mwd);

  plotOrientation(pi_manifold_mwd->times_raw(), pi_manifold_mwd->R_i_k(),
                  "Manifold MWD");

  // Quat RK4
  PreIntegrator::Ptr pi_quat_rk4 =
      std::make_shared<QuaternionPreIntegrationState>(
        gyroscope_noise_covariance,
        PreIntegrator::RungeKutta4);
  pi_quat_rk4->computeAbsolutes(true);
  preintegration_runner->process(pi_quat_rk4);

  plotOrientation(pi_quat_rk4->times_raw(), pi_quat_rk4->R_i_k(),
                  "Quat RK4");

  // Quat CG4
  PreIntegrator::Ptr pi_quat_cg4 =
      std::make_shared<QuaternionPreIntegrationState>(
        gyroscope_noise_covariance,
        PreIntegrator::CrouchGrossman4);
  pi_quat_cg4->computeAbsolutes(true);
  preintegration_runner->process(pi_quat_cg4);

  plotOrientation(pi_quat_cg4->times_raw(), pi_quat_cg4->R_i_k(),
                  "Quat CG4");


  // Map the stamped transformation vector to time and Rotation matrix vectors.
  std::vector<real_t> times_ref;
  std::vector<Matrix3> R_ref_list;
  for (auto entry: T_ref_list)
  {
    times_ref.push_back(static_cast<real_t>(entry.first) * 1e-9);
    R_ref_list.push_back(entry.second.getRotationMatrix());
  }

  plotOrientation(times_ref, R_ref_list, "Reference");
  VLOG(1) << "Processed integration steps: " << times_ref.size();

}


// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::runDriftEvaluationRuns(
    size_t runs,
    const Matrix3& gyroscope_noise_covariance,
    RandomVectorSampler<3>::Ptr accel_noise,
    RandomVectorSampler<3>::Ptr gyro_noise
    )
{
  MatrixX results(8, runs);

  for (size_t i = 0; i < runs; ++i)
  {
    PreIntegrationRunner::Ptr preintegration_runner = getPreIntegrationRunner(
                                                        accel_noise,
                                                        gyro_noise);

    real_t start = trajectory_->t_min();
    real_t end = trajectory_->t_max() - 5;

    std::vector<real_t> errors = runDriftEvaluation(preintegration_runner,
                                                       gyroscope_noise_covariance,
                                                       start,
                                                       end);
    for (size_t j = 0; j < errors.size(); ++j)
    {
      results(j, i) = errors[j];
    }
  }

  VectorX mean = results.rowwise().mean();
  VLOG(1) << "Manifold Fwd: " << mean(0) << "\n";
  VLOG(1) << "Manifold Mid: " << mean(1) << "\n";
  VLOG(1) << "Quaternion Fwd: " << mean(2) << "\n";
  VLOG(1) << "Quaternion Mid: " << mean(3) << "\n";
  VLOG(1) << "RK3: " << mean(4) << "\n";
  VLOG(1) << "RK4: " << mean(5) << "\n";
  VLOG(1) << "CG3: " << mean(6) << "\n";
  VLOG(1) << "CG4: " << mean(7) << "\n";

  plt::figure("drift_eval_boxplot");
  plt::boxplot(results, {
                 "Manifold Fwd",
                 "Manifold Mid",
                 "Quaternion Fwd",
                 "Quaternion Mid",
                 "RK3",
                 "RK4",
                 "CG3",
                 "CG4"
               });
  plt::title("Rotational Drift Cumulative Error");
  plt::show();
}

// -----------------------------------------------------------------------------
std::vector<real_t> PreIntegrationEvaluationNode::runDriftEvaluation(
    const PreIntegrationRunner::Ptr& preintegration_runner,
    const Matrix3& gyroscope_noise_covariance,
    real_t start,
    real_t end,
    bool plot)
{
  std::vector<real_t> errors;

  //! Evalutes an orientation error wrt. the reference trajectory.
  auto evaluateOrientationError = [this](const std::vector<real_t> times,
                                  const std::vector<Matrix3> est) -> real_t
  {
    real_t error = 0;
    for (size_t i = 0; i < est.size(); ++i)
    {
      Quaternion q1 = Quaternion::fromApproximateRotationMatrix(est[i]);
      Quaternion q2(trajectory_->orientation(times[i]));
      error += q1.getDisparityAngle(q2) / static_cast<real_t>(est.size());
    }

    return error;
  };

  //! Run preintegration from start to end on a given pre integrator
  //! Returns a pair containing the preintegrator itself and the float error.
  auto preIntegrate = [&start, &end, &evaluateOrientationError,
                      &preintegration_runner](PreIntegrator::Ptr pi)
  {
    pi->computeAbsolutes(true);
    preintegration_runner->process(pi, true, start, end);

    return std::make_pair(pi, evaluateOrientationError(pi->times_raw(), pi->R_i_k()));
  };

  VLOG(1) << "Simulate single run for integration drift.";
  ThreadPool pool(FLAGS_num_threads);
  std::vector<std::future<std::pair<PreIntegrator::Ptr, real_t>>> results_future;

  // 1) Manifold Fwd
  results_future.emplace_back(
        pool.enqueue([&gyroscope_noise_covariance, &preIntegrate]
  {
    return preIntegrate(std::make_shared<ManifoldPreIntegrationState>(
                          gyroscope_noise_covariance,
                          PreIntegrator::FirstOrderForward));
  }));

  // 2) Manifold Mid
  results_future.emplace_back(
        pool.enqueue([&gyroscope_noise_covariance, &preIntegrate]
  {
    return preIntegrate(std::make_shared<ManifoldPreIntegrationState>(
                          gyroscope_noise_covariance,
                          PreIntegrator::FirstOrderMidward));
  }));

  // 3) Quaternion FWD
  results_future.emplace_back(
        pool.enqueue([&gyroscope_noise_covariance, &preIntegrate]
  {
    return preIntegrate(std::make_shared<QuaternionPreIntegrationState>(
                          gyroscope_noise_covariance,
                          QuaternionPreIntegrationState::FirstOrderForward));
  }));

  // 4) Quaternion Mid
  results_future.emplace_back(
        pool.enqueue([&gyroscope_noise_covariance, &preIntegrate]
  {
    return preIntegrate(std::make_shared<QuaternionPreIntegrationState>(
                          gyroscope_noise_covariance,
                          QuaternionPreIntegrationState::FirstOrderMidward));
  }));

  // 5) Quaternion RK3
  results_future.emplace_back(
        pool.enqueue([&gyroscope_noise_covariance, &preIntegrate]
  {
    return preIntegrate(std::make_shared<QuaternionPreIntegrationState>(
                          gyroscope_noise_covariance,
                          QuaternionPreIntegrationState::RungeKutta3));
  }));


  // 6) Quaternion RK4
  results_future.emplace_back(
        pool.enqueue([&gyroscope_noise_covariance, &preIntegrate]
  {
    return preIntegrate(std::make_shared<QuaternionPreIntegrationState>(
                          gyroscope_noise_covariance,
                          QuaternionPreIntegrationState::RungeKutta4));
  }));

  // 7) Quaternion CG3
  results_future.emplace_back(
        pool.enqueue([&gyroscope_noise_covariance, &preIntegrate]
  {
    return preIntegrate(std::make_shared<QuaternionPreIntegrationState>(
                          gyroscope_noise_covariance,
                          QuaternionPreIntegrationState::CrouchGrossman3));
  }));

  // 8) Quaternion CG4
  results_future.emplace_back(
        pool.enqueue([&gyroscope_noise_covariance, &preIntegrate]
  {
    return preIntegrate(std::make_shared<QuaternionPreIntegrationState>(
                          gyroscope_noise_covariance,
                          QuaternionPreIntegrationState::CrouchGrossman4));
  }));


  // Extract the results of the simulations
  std::vector<PreIntegrator::Ptr> result_integrators;
  for (auto iter = results_future.begin(); iter != results_future.end(); ++iter)
  {
    auto value = iter->get();

    result_integrators.push_back(value.first);
    errors.push_back(value.second);
  }

  if (plot)
  {
    plotOrientation(result_integrators[0]->times_raw(),
        result_integrators[0]->R_i_k(), "ManifoldFwd", trajectory_);
    plotOrientation(result_integrators[1]->times_raw(),
        result_integrators[1]->R_i_k(), "ManifoldMid");
    plotOrientation(result_integrators[2]->times_raw(),
        result_integrators[2]->R_i_k(), "QuatFwd");
    plotOrientation(result_integrators[3]->times_raw(),
        result_integrators[3]->R_i_k(), "QuatMid");
    plotOrientation(result_integrators[4]->times_raw(),
        result_integrators[4]->R_i_k(), "QuatRK3");
    plotOrientation(result_integrators[5]->times_raw(),
        result_integrators[5]->R_i_k(), "QuatRK4");
    plotOrientation(result_integrators[6]->times_raw(),
        result_integrators[6]->R_i_k(), "QuatCG3");
    plotOrientation(result_integrators[7]->times_raw(),
        result_integrators[7]->R_i_k(), "QuatCG4");

    plotOrientationError(result_integrators[0]->times_raw(),
        result_integrators[0]->R_i_k(), "ManifoldFwd", trajectory_);
    plotOrientationError(result_integrators[1]->times_raw(),
        result_integrators[1]->R_i_k(), "ManifoldMid", trajectory_);
    plotOrientationError(result_integrators[2]->times_raw(),
        result_integrators[2]->R_i_k(), "QuatFwd", trajectory_);
    plotOrientationError(result_integrators[3]->times_raw(),
        result_integrators[3]->R_i_k(), "QuatMid", trajectory_);
    plotOrientationError(result_integrators[4]->times_raw(),
        result_integrators[4]->R_i_k(), "QuatRK3", trajectory_);
    plotOrientationError(result_integrators[5]->times_raw(),
        result_integrators[5]->R_i_k(), "QuatRK4", trajectory_);
    plotOrientationError(result_integrators[6]->times_raw(),
        result_integrators[6]->R_i_k(), "QuatCG3", trajectory_);
    plotOrientationError(result_integrators[7]->times_raw(),
        result_integrators[7]->R_i_k(), "QuatCG4", trajectory_);
  }

  return errors;
}

// -----------------------------------------------------------------------------
PreIntegratorMonteCarlo::Ptr PreIntegrationEvaluationNode::runManifoldCorruptedMc(
    const PreIntegrationRunner::Ptr& preintegration_runner,
    PreIntegrator::IntegratorType integrator_type,
    const std::string& name,
    const Matrix3& gyroscope_noise_covariance,
    real_t start,
    real_t end)
{
  VLOG(1) << "Monte Carlo Simulation [Manifoold: " + name + "]";
  PreIntegratorFactory::Ptr preintegrator_factory(
        std::make_shared<ManifoldPreIntegrationFactory>(gyroscope_noise_covariance,
                                                        integrator_type));
  PreIntegratorMonteCarlo::Ptr mc(
        std::make_shared<PreIntegratorMonteCarlo>(preintegration_runner,
                                                  preintegrator_factory,
                                                  FLAGS_num_threads));
  mc->simulate(FLAGS_monte_carlo_runs, start, end);

  plotCovarianceResults({mc->covariances()},
                        {"MC" + name});

  return mc;
}

// -----------------------------------------------------------------------------
PreIntegratorMonteCarlo::Ptr PreIntegrationEvaluationNode::runQuaternionMc(
    const PreIntegrationRunner::Ptr& preintegration_runner,
    PreIntegrator::IntegratorType integrator_type,
    const std::string& name,
    const Matrix3& gyroscope_noise_covariance,
    real_t start,
    real_t end)
{
  VLOG(1) << "Monte Carlo Simulation [Quaternion: " + name + "]";
  PreIntegratorFactory::Ptr preintegrator_factory(
        std::make_shared<QuaternionPreIntegrationFactory>(
          gyroscope_noise_covariance, integrator_type));
  PreIntegratorMonteCarlo::Ptr mc(
        std::make_shared<PreIntegratorMonteCarlo>(preintegration_runner,
                                                  preintegrator_factory,
                                                  FLAGS_num_threads));
  mc->simulate(FLAGS_monte_carlo_runs, start, end);

  plotCovarianceResults({mc->covariances()},
                        {"MC" + name});

  return mc;
}

// -----------------------------------------------------------------------------
ManifoldPreIntegrationState::Ptr PreIntegrationEvaluationNode::runManifoldCorrupted(
    const PreIntegrationRunner::Ptr& preintegration_runner,
    PreIntegrator::IntegratorType integrator_type,
    const std::string& name,
    const Matrix3& gyroscope_noise_covariance,
    real_t start,
    real_t end,
    const PreIntegratorMonteCarlo::Ptr& mc,
    bool simplified_covariance)
{
  VLOG(1) << "Reference Estimates [ManifoldPreIntegrator:Corrupted]";
  PreIntegratorFactory::Ptr preintegrator_factory(
        std::make_shared<ManifoldPreIntegrationFactory>(gyroscope_noise_covariance,
                                                        integrator_type,
                                                        simplified_covariance));

  ManifoldPreIntegrationState::Ptr est_integrator = preintegrator_factory->get();

  preintegration_runner->process(est_integrator,
                                 true,
                                 start,
                                 end);
  // Result visualization:
  plotCovarianceResults({est_integrator->covariance_i_k()},
                        {"Est" + name});

  if (mc)
  {
    plotCovarianceError(mc->covariances(),
                        est_integrator->covariance_i_k(),
                        name);
  }

  // Show the timers for the integration.
  if (FLAGS_time_integration)
  {
    VLOG(1) << est_integrator->timers_;
  }

  return est_integrator;
}

// -----------------------------------------------------------------------------
QuaternionPreIntegrationState::Ptr PreIntegrationEvaluationNode::runQuaternion(
    const PreIntegrationRunner::Ptr& preintegration_runner,
    PreIntegrator::IntegratorType integrator_type,
    const std::string& name,
    const Matrix3& gyroscope_noise_covariance,
    real_t start,
    real_t end,
    const PreIntegratorMonteCarlo::Ptr& mc)
{
  VLOG(1) << "Reference Estimates [Quaternion]";
  PreIntegratorFactory::Ptr preintegrator_factory(
        std::make_shared<QuaternionPreIntegrationFactory>(gyroscope_noise_covariance,
                                                          integrator_type));

  QuaternionPreIntegrationState::Ptr est_integrator = preintegrator_factory->get();

  preintegration_runner->process(est_integrator,
                                 true,
                                 start,
                                 end);
  // Result visualization:
  plotCovarianceResults({est_integrator->covariance_i_k()},
                        {"Est" + name});

  if (mc)
  {
    plotCovarianceError(mc->covariances(),
                        est_integrator->covariance_i_k(),
                        name);
  }

  // Show the timers for the integration.
  if (FLAGS_time_integration)
  {
    VLOG(1) << est_integrator->timers_;
  }

  return est_integrator;
}

// -----------------------------------------------------------------------------
PreIntegrationRunner::Ptr PreIntegrationEvaluationNode::getPreIntegrationRunner(
    RandomVectorSampler<3>::Ptr accel_noise,
    RandomVectorSampler<3>::Ptr gyro_noise)
{
  loadTrajectory();
  real_t start = trajectory_->t_min();
  real_t end = trajectory_->t_max();

  VLOG(1) << "Initialize scenario";
  TrajectorySimulator::Ptr scenario = std::make_shared<SplineTrajectorySimulator>(trajectory_);

  VLOG(1) << "Initialize scenario runner";

  ImuBiasSimulator::Ptr bias;
  if (parameters_.imu_bias_type == "continuous")
  {
    bias = generateImuBias(start, end, parameters_.imu_bias_type,
                           Vector3::Ones() * parameters_.imu_acc_bias_noise_density,
                           Vector3::Ones() * parameters_.imu_gyr_bias_noise_density);
  }
  else
  {
    bias = generateImuBias(start, end, parameters_.imu_bias_type,
                           Vector3::Ones() * parameters_.imu_acc_bias_const,
                           Vector3::Ones() * parameters_.imu_gyr_bias_const);
  }
  ImuSimulator::Ptr scenario_runner =
      std::make_shared<ImuSimulator>(scenario,
                                       bias,
                                       accel_noise,
                                       gyro_noise,
                                       1.0 / parameters_.imu_sampling_time,
                                       1.0 / parameters_.imu_sampling_time,
                                       parameters_.gravity_magnitude);

  PreIntegrationRunner::Ptr preintegration_runner =
      std::make_shared<PreIntegrationRunner>(
        scenario_runner,
        parameters_.imu_sampling_time,
        parameters_.camera_sampling_time);

  preintegration_runner->setInitialOrientation(trajectory_->orientation(start));

  return preintegration_runner;
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::loadTrajectory()
{
  // A bspline fixed at 3rd order.
  trajectory_ = std::make_shared<BSplinePoseMinimalRotationVector>(
                  parameters_.trajectory_spline_order);
  // generate random
  if (parameters_.trajectory_source == "")
  {
    real_t start = parameters_.trajectory_start_time;
    real_t end = parameters_.trajectory_end_time;

    VLOG(1) << "Generating random trajectory of " << (end - start) << "seconds";

    MatrixX points(6, parameters_.trajectory_num_interpolation_points);
    points.setRandom();
    // make translations significanter
    points.block(0, 0, 3, parameters_.trajectory_num_interpolation_points) *=
        parameters_.trajectory_translation_factor;

    // Rotation Multiplier
    points.block(3, 0, 3, parameters_.trajectory_num_interpolation_points) *=
        parameters_.trajectory_rotation_factor;

    VectorX times;
    times.setLinSpaced(parameters_.trajectory_num_interpolation_points, start, end);

    trajectory_->initPoseSpline3(times,
                                 points,
                                 parameters_.trajectory_num_segments,
                                 parameters_.trajectory_lambda);
  }
  // load from file
  else
  {
    CHECK_EQ(false, true) << "Not implemented.";
  }

  // Display the trajectory?
  if (FLAGS_show_trajectory)
  {
    showTrajectory();
  }
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::showTrajectory()
{
  splines_visualizer_->plotSpline(*trajectory_);
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::plotCovarianceResults(
    std::initializer_list<const std::vector<Matrix3>> covariances_vectors,
    std::initializer_list<const std::string> labels)
{

  plt::figure("covariances");
  auto label = labels.begin();
  for(auto elem: covariances_vectors)
  {
    Eigen::Matrix<real_t, 3, Eigen::Dynamic> v(3, elem.size());
    for (size_t i = 0; i < elem.size(); ++i)
    {
      v.col(i) = elem[i].diagonal();
    }
    plt::subplot(3, 1, 1);
    plt::labelPlot(*label, v.row(0));
    plt::legend();
    plt::subplot(3, 1, 2);
    plt::labelPlot(*label, v.row(1));
    plt::subplot(3, 1, 3);
    plt::labelPlot(*label, v.row(2));

    ++label;
  }
  plt::show(false);
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::plotCovarianceError(
    const std::vector<Matrix3> ref,
    const std::vector<Matrix3> est,
    const std::string& label)
{
  CHECK_EQ(ref.size(), est.size());

  plt::figure("covariance_offsets");
  Eigen::Matrix<real_t, 3, Eigen::Dynamic> v(3, ref.size());
  for (size_t i = 0; i < ref.size(); ++i)
  {
    v.col(i) = (ref[i].diagonal() - est[i].diagonal()).cwiseAbs();
  }
  plt::subplot(3, 1, 1);
  plt::labelPlot(label, v.row(0));
  plt::legend();
  plt::subplot(3, 1, 2);
  plt::labelPlot(label, v.row(1));
  plt::subplot(3, 1, 3);
  plt::labelPlot(label, v.row(2));

  plt::show(false);

  plt::figure("covariance_distance");
  Eigen::Matrix<real_t, 1, Eigen::Dynamic> dist(1, ref.size());
  for (size_t i = 0; i < ref.size(); ++i)
  {
    dist(i) = (ref[i] - est[i]).norm();
  }
  // total distance output:
  VLOG(1) << "Covariance Offset [" + label + "]: " << dist.norm();

  plt::figure("covariance_distance");
  plt::labelPlot(label, dist);
  plt::legend();
  plt::show(false);
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::shutdown()
{
  // Blocking destructor to keep figures open.
  plt::figure();
  plt::show();
}

} // namespace ze

// -----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  VLOG(1) << "Create PreIntegration Evaluation Node.";
  ze::PreIntegrationEvaluationNode node;

  if (FLAGS_run_type == "drift")
  {
    node.runDriftEvaluationMain();
  }
  else if (FLAGS_run_type == "covariance")
  {
    node.runCovarianceMonteCarloMain();
  }
  else if (FLAGS_run_type == "real")
  {
    node.runRealDatasetMain();
  }
  else
  {
    throw std::runtime_error("Invalid run type");
  }

  VLOG(1) << "Finish Processing.";
  node.shutdown();
  VLOG(1) << "Node terminated cleanly.";
  return 0;
}
