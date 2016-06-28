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

#include <ze/imu_evaluation/pre_integration_runner.hpp>
#include <ze/imu_evaluation/pre_integration_mc_runner.hpp>
#include <ze/imu_evaluation/scenario_runner.hpp>
#include <ze/imu_evaluation/scenario.hpp>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/common/csv_trajectory.h>
#include <ze/imu_evaluation/imu_bias.hpp>
#include <ze/splines/viz_splines.hpp>
#include <ze/visualization/viz_ros.h>
#include <ze/matplotlib/matplotlibcpp.hpp>
#include <ze/splines/rotation_vector.hpp>
#include <ze/imu_evaluation/manifold_pre_integrator.hpp>
#include <ze/imu_evaluation/quaternion_pre_integrator.hpp>

DEFINE_string(trajectory_source, "", "Path to file to load curve from, default: generate random curve");

DEFINE_int32(monte_carlo_runs, 10, "Number of monte-carlo simulations.");

// For an empty curve_source parameters to generate a random trajectory:
DEFINE_double(trajectory_length, 30.0, "The length of the curve in seconds");
DEFINE_int32(trajectory_interpolation_points, 100, "Number of interpolation to randomize");
DEFINE_int32(trajectory_num_segments, 100, "Number of spline segments to fit the interpolation points.");
DEFINE_double(trajectory_lambda, 1e-5, "The regularizing smoothing factor used to fit the trajectory to the interpolation curve. Smaller values give more aggressive curves.");

// Imu Simulation Parameters:
DEFINE_double(imu_sampling_time, 0.005, "Sampling time of the IMU.");
DEFINE_double(camera_sampling_time, 0.1, "Sampling time of the Camera, only used to split the pre-integration steps.");
DEFINE_double(accelerometer_noise_bandwidth_hz, 200, "Bandwidth of accelerometer noise");
DEFINE_double(gyroscope_noise_bandwidth_hz, 200, "Bandwith of gyroscope noise");
DEFINE_double(accelerometer_noise_density, 0, "Noise density of accelerometer.");
DEFINE_double(gyroscope_noise_density, 0, "Noise density of gyroscope");

// Im u Bias Simulation
DEFINE_string(imu_bias_type, "constant", "Bias model of the imu");
DEFINE_double(imu_acc_bias_noise_density, 0, "Noise density of continuous-time accelerometer bias.");
DEFINE_double(imu_gyr_bias_noise_density, 0, "Noise density of continuous-time gyroscope bias.");

DEFINE_double(imu_acc_bias_const, 0, "Value of constant accelerometer bias.");
DEFINE_double(imu_gyr_bias_const, 0, "Value of constant gyroscope bias.");

// A series of visualization controls:
DEFINE_bool(show_trajectory, true, "Show the trajectory that was loaded / generated.");
DEFINE_int32(num_threads, 16, "Number of threads to take for the monte-carlo simulations");


namespace ze {

//! A parameter structure to regroup all simulation parameters.
struct ImuPreIntegrationParameters
{
  // The Imu Model and Noise Parameters:
  FloatType imu_sampling_time;
  FloatType camera_sampling_time;
  FloatType accel_noise_bandwidth_hz;
  FloatType gyro_noise_bandwidth_hz;
  FloatType accel_noise_density;
  FloatType gyro_noise_density;
  Vector3 gravity;

  // Imu Bias Model
  std::string imu_bias_type;

  // Parameters of continous-time bias model:
  FloatType imu_acc_bias_noise_density;
  FloatType imu_gyr_bias_noise_density;

  // Parameters of constant bias model:
  FloatType imu_acc_bias_const;
  FloatType imu_gyr_bias_const;

  // Trajectory Generation Parameters:
  std::string trajectory_source;
  FloatType trajectory_start_time;
  FloatType trajectory_end_time;
  int trajectory_num_interpolation_points;
  int trajectory_num_segments;
  FloatType trajectory_lambda;
  int trajectory_spline_order = 5;

  //! Initialize a parameter structure from gflags
  static ImuPreIntegrationParameters fromGFlags()
  {
    ImuPreIntegrationParameters p;

    p.imu_sampling_time = FLAGS_imu_sampling_time;
    p.camera_sampling_time = FLAGS_camera_sampling_time;
    p.accel_noise_bandwidth_hz = FLAGS_accelerometer_noise_bandwidth_hz;
    p.gyro_noise_bandwidth_hz = FLAGS_gyroscope_noise_bandwidth_hz;
    p.gravity = Vector3(0, 0, -9.81);

    p.accel_noise_density = FLAGS_accelerometer_noise_density;
    p.gyro_noise_density = FLAGS_gyroscope_noise_density;

    p.trajectory_source = FLAGS_trajectory_source;
    p.trajectory_start_time = 0.0;
    p.trajectory_end_time = FLAGS_trajectory_length;
    p.trajectory_num_interpolation_points = FLAGS_trajectory_interpolation_points;
    p.trajectory_num_segments = FLAGS_trajectory_num_segments;
    p.trajectory_lambda = FLAGS_trajectory_lambda;

    p.imu_bias_type = FLAGS_imu_bias_type;

    p.imu_acc_bias_noise_density = FLAGS_imu_acc_bias_noise_density;
    p.imu_gyr_bias_noise_density = FLAGS_imu_gyr_bias_noise_density;

    p.imu_acc_bias_const = FLAGS_imu_acc_bias_const;
    p.imu_gyr_bias_const = FLAGS_imu_gyr_bias_const;

    return p;
  }

  //! Initialize a parameter structure given an imu yaml file.
  static /*ImuPreIntegrationParameters*/ void fromImuModel()
  {
    //! @todo
  }
};

// -----------------------------------------------------------------------------
class PreIntegrationEvaluationNode
{
public:
  PreIntegrationEvaluationNode();
  ~PreIntegrationEvaluationNode() = default;

  //! Return the ImuBias for model and parameters specified in the GFlags.
  ImuBias::Ptr imuBias(FloatType start, FloatType end);

  //! Simulation Runners
  PreIntegratorMonteCarlo::Ptr runManifoldClean(
      PreIntegrationRunner::Ptr preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      const std::vector<Matrix3>* D_R_i_k_reference,
      FloatType start,
      FloatType end);

  PreIntegratorMonteCarlo::Ptr runManifoldCorrupted(
      PreIntegrationRunner::Ptr preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      FloatType start,
      FloatType end);

  PreIntegratorMonteCarlo::Ptr runQuaternion(
      PreIntegrationRunner::Ptr preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      FloatType start,
      FloatType end);

  //! Runs a single noisy integration on all channels and shows plots of the
  //! estimates.
  //! Returns the error offset (rotation only) for the different methods
  std::vector<FloatType> runDriftEvaluation(
      PreIntegrationRunner::Ptr preintegration_runner,
      const Matrix3& gyroscope_noise_covariance,
      FloatType start,
      FloatType end,
      bool plot = false);

  void runDriftEvaluationRuns(size_t runs,
                              const Matrix3& gyroscope_noise_covariance,
                              RandomVectorSampler<3>::Ptr accel_noise,
                              RandomVectorSampler<3>::Ptr gyro_noise);

  //! Get a preintegration runner and generate a trajectory.
  PreIntegrationRunner::Ptr getPreIntegrationRunner(
      RandomVectorSampler<3>::Ptr accel_noise,
      RandomVectorSampler<3>::Ptr gyro_noise);

  //! Given the GFLags configuration loads and returns a bspline representing
  //! a trajectory.
  void loadTrajectory();

  //! Show the trajectory that was loaded or generated.
  void showTrajectory();

  //! Generate a series of plots that show the results
  void plotCovarianceResults(
      std::initializer_list<const std::vector<Matrix3>> covariances_vectors,
      std::initializer_list<const std::string> labels);

  //! Generate a series of plots that show the results
  void plotCovarianceError(const std::vector<Matrix3> ref,
                             const std::vector<Matrix3> est,
                             const std::string& label);

  //! Plot the estimated / integrated vector of rotation matrices.
  void plotOrientation(const std::vector<FloatType>& times,
                       const std::vector<Matrix3>& R_i,
                       const std::string& label,
                       const bool plot_reference = false);

  void plotOrientationError(const std::vector<FloatType>& times,
                              const std::vector<Matrix3>& est,
                              const std::string& label);

  //! Plot the measurements used to obtain a preintegration state.
  void plotImuMeasurements(const std::vector<FloatType>& times,
                           const ImuAccGyrContainer& measurements,
                           const ImuAccGyrContainer& measurements2);

  void shutdown();

private:
  ImuPreIntegrationParameters parameters_;

  std::shared_ptr<BSplinePoseMinimalRotationVector> trajectory_;

  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<SplinesVisualizer> splines_visualizer_;
};

// -----------------------------------------------------------------------------
PreIntegrationEvaluationNode::PreIntegrationEvaluationNode()
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

  PreIntegrationRunner::Ptr preintegration_runner = getPreIntegrationRunner(
                                                      accel_noise,
                                                      gyro_noise);

  FloatType start = trajectory_->t_min();
  FloatType end = trajectory_->t_max() - 5;

  /////// Different Monte Carlo Simulation runs:

  // 1) Manifold Integration Forward
  PreIntegratorMonteCarlo::Ptr mc_corrupted_fwd =
      runManifoldCorrupted(preintegration_runner,
                           PreIntegrator::FirstOrderForward,
                           "Manifold Corr FWD",
                           gyroscope_noise_covariance,
                           start,
                           end);

  // 2) Manifold Integration Midward
  PreIntegratorMonteCarlo::Ptr mc_corrupted_mid =
      runManifoldCorrupted(preintegration_runner,
                           PreIntegrator::FirstOrderMidward,
                           "Manifold Corr MWD",
                           gyroscope_noise_covariance,
                           start,
                           end);

  // 3) Manifold Integration with clean orientation estimates
  // Use the corrupted MC simulator to get an unperturbed orientation estimate.
  PreIntegrator::Ptr actual_integrator = mc_corrupted->preintegrateActual(start,
                                                                          end);
  runManifoldClean(preintegration_runner,
                   gyroscope_noise_covariance,
                   &actual_integrator->D_R_i_k(),
                   start,
                   end);

  // 4) Quaternion Integration: Forward Integration
  PreIntegratorMonteCarlo::Ptr mc_quat_fwd =
      runQuaternion(preintegration_runner,
                    PreIntegrator::FirstOrderForward,
                    "Quat FWD",
                    gyroscope_noise_covariance,
                    start,
                    end);

  // 5) Quaternion Integration: Forward Integration
  PreIntegratorMonteCarlo::Ptr mc_quat_mid =
      runQuaternion(preintegration_runner,
                    PreIntegrator::FirstOrderMidward,
                    "Quat MWD",
                    gyroscope_noise_covariance,
                    start,
                    end);

  // 6) Quaternion Integration: RK3
  PreIntegratorMonteCarlo::Ptr mc_quat_rk3 =
      runQuaternion(preintegration_runner,
                    PreIntegrator::RungeKutta3,
                    "RK3",
                    gyroscope_noise_covariance,
                    start,
                    end);

  // 7) Quaternion Integration: RK4
  PreIntegratorMonteCarlo::Ptr mc_quat_rk4 =
      runQuaternion(preintegration_runner,
                    PreIntegrator::RungeKutta4,
                    "RK4",
                    gyroscope_noise_covariance,
                    start,
                    end);

  /////// Evaluate drifts:
  runDriftEvaluationRuns(200, gyroscope_noise_covariance, accel_noise, gyro_noise);

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

    FloatType start = trajectory_->t_min();
    FloatType end = trajectory_->t_max() - 5;

    std::vector<FloatType> errors = runDriftEvaluation(preintegration_runner,
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
std::vector<FloatType> PreIntegrationEvaluationNode::runDriftEvaluation(
    PreIntegrationRunner::Ptr preintegration_runner,
    const Matrix3& gyroscope_noise_covariance,
    FloatType start,
    FloatType end,
    bool plot)
{
  std::vector<FloatType> errors;

  auto evaluateOrientationError = [this](const std::vector<FloatType> times,
                                  const std::vector<Matrix3> est) -> FloatType
  {
    FloatType error = 0;
    for (size_t i = 0; i < est.size(); ++i)
    {
      Quaternion q1(est[i]);
      Quaternion q2(trajectory_->orientation(times[i]));
      error += q1.getDisparityAngle(q2) / static_cast<FloatType>(est.size());
    }

    return error;
  };


  VLOG(1) << "Simulate single run for integration drift.";

  // 1) Manifold Fwd
  PreIntegrator::Ptr pi_manifold_fwd =
      std::make_shared<ManifoldPreIntegrationState>(
        gyroscope_noise_covariance,
        PreIntegrator::FirstOrderForward);

  preintegration_runner->process(pi_manifold_fwd,
                                 true,
                                 start,
                                 end);
  errors.push_back(
        evaluateOrientationError(pi_manifold_fwd->times_raw(),
                                 pi_manifold_fwd->R_i_k()));

  // 2) Manifold Mid
  PreIntegrator::Ptr pi_manifold_mid =
      std::make_shared<ManifoldPreIntegrationState>(
        gyroscope_noise_covariance,
        PreIntegrator::FirstOrderMidward);

  preintegration_runner->process(pi_manifold_mid,
                                 true,
                                 start,
                                 end);
  errors.push_back(
        evaluateOrientationError(pi_manifold_mid->times_raw(),
                                 pi_manifold_mid->R_i_k()));

  // 3) Quaternion FWD
  PreIntegrator::Ptr pi_quat_fwd =
      std::make_shared<QuaternionPreIntegrationState>(
        gyroscope_noise_covariance,
        QuaternionPreIntegrationState::FirstOrderForward);
  preintegration_runner->process(pi_quat_fwd,
                                 true,
                                 start,
                                 end);
  errors.push_back(
        evaluateOrientationError(pi_quat_fwd->times_raw(),
                                 pi_quat_fwd->R_i_k()));

  // 4) Quaternion Mid
  PreIntegrator::Ptr pi_quat_mid =
      std::make_shared<QuaternionPreIntegrationState>(
        gyroscope_noise_covariance,
        QuaternionPreIntegrationState::FirstOrderMidward);
  preintegration_runner->process(pi_quat_mid,
                                 true,
                                 start,
                                 end);
  errors.push_back(
        evaluateOrientationError(pi_quat_mid->times_raw(),
                                 pi_quat_mid->R_i_k()));

  // 5) Quaternion RK3
  PreIntegrator::Ptr pi_quat_rk3 =
      std::make_shared<QuaternionPreIntegrationState>(
        gyroscope_noise_covariance,
        QuaternionPreIntegrationState::RungeKutta3);
  preintegration_runner->process(pi_quat_rk3,
                                 true,
                                 start,
                                 end);
  errors.push_back(
        evaluateOrientationError(pi_quat_rk3->times_raw(),
                                 pi_quat_rk3->R_i_k()));

  // 6) Quaternion RK4
  PreIntegrator::Ptr pi_quat_rk4 =
      std::make_shared<QuaternionPreIntegrationState>(
        gyroscope_noise_covariance,
        QuaternionPreIntegrationState::RungeKutta4);
  preintegration_runner->process(pi_quat_rk4,
                                 true,
                                 start,
                                 end);
  errors.push_back(
        evaluateOrientationError(pi_quat_rk4->times_raw(),
                                 pi_quat_rk4->R_i_k()));

  // 7) Quaternion CG3
  PreIntegrator::Ptr pi_quat_cg3 =
      std::make_shared<QuaternionPreIntegrationState>(
        gyroscope_noise_covariance,
        QuaternionPreIntegrationState::CrouchGrossman3);
  preintegration_runner->process(pi_quat_cg3,
                                 true,
                                 start,
                                 end);
  errors.push_back(
        evaluateOrientationError(pi_quat_cg3->times_raw(),
                                 pi_quat_cg3->R_i_k()));

  // 8) Quaternion CG4
  PreIntegrator::Ptr pi_quat_cg4 =
      std::make_shared<QuaternionPreIntegrationState>(
        gyroscope_noise_covariance,
        QuaternionPreIntegrationState::CrouchGrossman4);
  preintegration_runner->process(pi_quat_cg4 ,
                                 true,
                                 start,
                                 end);
  errors.push_back(
        evaluateOrientationError(pi_quat_cg4->times_raw(),
                                 pi_quat_cg4->R_i_k()));

  if (plot)
  {
    plotOrientation(pi_manifold_fwd->times_raw(), pi_manifold_fwd->R_i_k(), "ManifoldFwd", true);
    plotOrientation(pi_manifold_mid->times_raw(), pi_manifold_mid->R_i_k(), "ManifoldMid");
    plotOrientation(pi_quat_fwd->times_raw(), pi_quat_fwd->R_i_k(), "QuatFwd");
    plotOrientation(pi_quat_mid->times_raw(), pi_quat_mid->R_i_k(), "QuatMid");
    plotOrientation(pi_quat_rk3->times_raw(), pi_quat_rk3->R_i_k(), "QuatRK3");
    plotOrientation(pi_quat_rk4->times_raw(), pi_quat_rk4->R_i_k(), "QuatRK4");
    plotOrientation(pi_quat_cg3->times_raw(), pi_quat_cg3->R_i_k(), "QuatCG3");
    plotOrientation(pi_quat_cg4->times_raw(), pi_quat_cg4->R_i_k(), "QuatCG4");

    plotOrientationError(pi_manifold_fwd->times_raw(), pi_manifold_fwd->R_i_k(), "ManifoldFwd");
    plotOrientationError(pi_manifold_mid->times_raw(), pi_manifold_mid->R_i_k(), "ManifoldMid");
    plotOrientationError(pi_quat_fwd->times_raw(), pi_quat_fwd->R_i_k(), "QuatFwd");
    plotOrientationError(pi_quat_mid->times_raw(), pi_quat_mid->R_i_k(), "QuatMid");
    plotOrientationError(pi_quat_rk3->times_raw(), pi_quat_rk3->R_i_k(), "QuatRK3");
    plotOrientationError(pi_quat_rk4->times_raw(), pi_quat_rk4->R_i_k(), "QuatRK4");
    plotOrientationError(pi_quat_cg3->times_raw(), pi_quat_cg3->R_i_k(), "QuatCG3");
    plotOrientationError(pi_quat_cg4->times_raw(), pi_quat_cg4->R_i_k(), "QuatCG4");
  }

  return errors;
}


// -----------------------------------------------------------------------------
PreIntegratorMonteCarlo::Ptr PreIntegrationEvaluationNode::runManifoldClean(
    PreIntegrationRunner::Ptr preintegration_runner,
    PreIntegrator::IntegratorType integrator_type,
    const std::string& name,
    const Matrix3& gyroscope_noise_covariance,
    const std::vector<Matrix3>* D_R_i_k_reference,
    FloatType start,
    FloatType end)
{
  VLOG(1) << "Monte Carlo Simulation [ManifoldPreIntegrator:Clean]";
  PreIntegratorFactory::Ptr preintegrator_factory_clean(
        std::make_shared<ManifoldPreIntegrationFactory>(gyroscope_noise_covariance,
                                                        integrator_type,
                                                        D_R_i_k_reference));
  PreIntegratorMonteCarlo::Ptr mc(
        std::make_shared<PreIntegratorMonteCarlo>(
          preintegration_runner,
          preintegrator_factory_clean,
          FLAGS_num_threads));
  mc->simulate(FLAGS_monte_carlo_runs, start, end);

  ManifoldPreIntegrationState::Ptr est_integrator =
      mc->preintegrateCorrupted(start, end);

  plotCovarianceResults({mc->covariances(),
                         est_integrator->covariance_i_k()},
                         {"MC" + name, "Est" + name});

  plotCovarianceError(mc->covariances(),
                      est_integrator->covariance_i_k(),
                      name);

  return mc;
}

// -----------------------------------------------------------------------------
PreIntegratorMonteCarlo::Ptr PreIntegrationEvaluationNode::runManifoldCorrupted(
    PreIntegrationRunner::Ptr preintegration_runner,
    PreIntegrator::IntegratorType integrator_type,
    const std::string& name,
    const Matrix3& gyroscope_noise_covariance,
    FloatType start,
    FloatType end)
{
  VLOG(1) << "Monte Carlo Simulation [ManifoldPreIntegrator:Corrupted]";
  PreIntegratorFactory::Ptr preintegrator_factory(
        std::make_shared<ManifoldPreIntegrationFactory>(gyroscope_noise_covariance,
                                                        integrator_type));
  PreIntegratorMonteCarlo::Ptr mc(
        std::make_shared<PreIntegratorMonteCarlo>(preintegration_runner,
                                                  preintegrator_factory,
                                                  FLAGS_num_threads));
  mc->simulate(FLAGS_monte_carlo_runs, start, end);

  VLOG(1) << "Reference Estimates";
  // Corrupted integration:
  ManifoldPreIntegrationState::Ptr est_integrator = mc->preintegrateCorrupted(
                                                      start, end);
  // Result visualization:
  plotCovarianceResults({mc->covariances(),
                         est_integrator->covariance_i_k()},
                        {"MC" + name, "Est" + name});

  plotCovarianceError(mc->covariances(),
                        est_integrator->covariance_i_k(),
                        name);

  mc->clean();
  return mc;
}

// -----------------------------------------------------------------------------
PreIntegratorMonteCarlo::Ptr PreIntegrationEvaluationNode::runQuaternion(
    PreIntegrationRunner::Ptr preintegration_runner,
    PreIntegrator::IntegratorType integrator_type,
    const std::string& name,
    const Matrix3& gyroscope_noise_covariance,
    FloatType start,
    FloatType end)
{
  VLOG(1) << "Monte Carlo Simulation [Quaternion: Forward]";
  PreIntegratorFactory::Ptr preintegrator_factory(
        std::make_shared<QuaternionPreIntegrationFactory>(
          gyroscope_noise_covariance, integrator_type));
  PreIntegratorMonteCarlo::Ptr mc(
        std::make_shared<PreIntegratorMonteCarlo>(preintegration_runner,
                                                  preintegrator_factory,
                                                  FLAGS_num_threads));
  mc->simulate(FLAGS_monte_carlo_runs, start, end);

  VLOG(1) << "Reference Estimates";
  // Corrupted integration:
  QuaternionPreIntegrationState::Ptr est_integrator =
      mc->preintegrateCorrupted(start, end);

  // Result visualization:
  plotCovarianceResults({mc->covariances(),
                         est_integrator->covariance_i_k()},
                        {"MC" + name, "Est" + name});

  plotCovarianceError(mc->covariances(),
                        est_integrator->covariance_i_k(),
                        name);

  mc->clean();
  return mc;
}

// -----------------------------------------------------------------------------
PreIntegrationRunner::Ptr PreIntegrationEvaluationNode::getPreIntegrationRunner(
    RandomVectorSampler<3>::Ptr accel_noise,
    RandomVectorSampler<3>::Ptr gyro_noise)
{
  loadTrajectory();
  FloatType start = trajectory_->t_min();
  FloatType end = trajectory_->t_max();

  VLOG(1) << "Initialize scenario";
  Scenario::Ptr scenario = std::make_shared<SplineScenario>(trajectory_);

  VLOG(1) << "Initialize scenario runner";
  ScenarioRunner::Ptr scenario_runner =
      std::make_shared<ScenarioRunner>(scenario,
                                       imuBias(start, end),
                                       accel_noise,
                                       gyro_noise,
                                       parameters_.accel_noise_bandwidth_hz,
                                       parameters_.gyro_noise_bandwidth_hz,
                                       parameters_.gravity);

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
    FloatType start = parameters_.trajectory_start_time;
    FloatType end = parameters_.trajectory_end_time;

    VLOG(1) << "Generating random trajectory of " << (end - start) << "seconds";

    MatrixX points(6, parameters_.trajectory_num_interpolation_points);
    points.setRandom();
    // make translations significanter
    points.block(0, 0, 3, parameters_.trajectory_num_interpolation_points) *= 10;

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
ImuBias::Ptr PreIntegrationEvaluationNode::imuBias(FloatType start,
                                                   FloatType end)
{
  //! continuous bias model
  if (parameters_.imu_bias_type == "continuous")
  {

    return std::make_shared<ContinuousBias>(
          Vector3::Ones() * parameters_.imu_acc_bias_noise_density,
          Vector3::Ones() * parameters_.imu_gyr_bias_noise_density,
          start,
          end,
          1000); // This is an arbitrary value.
  }
  //! a simple constant bias
  else
  {
    Vector3 accel_bias = Vector3::Ones() * parameters_.imu_acc_bias_const;
    Vector3 gyro_bias = Vector3::Ones() * parameters_.imu_gyr_bias_const;

    return std::make_shared<ConstantBias>(accel_bias, gyro_bias);
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
    Eigen::Matrix<FloatType, 3, Eigen::Dynamic> v(3, elem.size());
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
  Eigen::Matrix<FloatType, 3, Eigen::Dynamic> v(3, ref.size());
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
  Eigen::Matrix<FloatType, 1, Eigen::Dynamic> dist(1, ref.size());
  for (size_t i = 0; i < ref.size(); ++i)
  {
    dist(i) = (ref[i] - est[i]).norm();
  }
  plt::figure("covariance_distance");
  plt::labelPlot(label, dist);
  plt::legend();
  plt::show(false);
}

//-----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::plotOrientation(
    const std::vector<FloatType>& times,
    const std::vector<Matrix3>& orientation,
    const std::string& label,
    const bool plot_reference)
{
  CHECK_EQ(times.size(), orientation.size());

  Eigen::Matrix<FloatType, 3, Eigen::Dynamic> points(3, orientation.size());
  Eigen::Matrix<FloatType, 3, Eigen::Dynamic> ref_points(3, orientation.size());

  for (size_t i = 0; i < orientation.size(); ++i)
  {
    ze::sm::RotationVector rv(orientation[i]);
    points.col(i) = rv.getParameters();
    ref_points.col(i) = trajectory_->eval(times[i]).tail<3>();
  }
  plt::figure("orientation");
  plt::subplot(3, 1, 1);
  plt::title("Orientation");
  plt::labelPlot(label, times, points.row(0), "r");
  if(plot_reference)
  {
    plt::labelPlot("reference", times, ref_points.row(0), "b");
  }
  plt::legend();

  plt::subplot(3, 1, 2);
  plt::labelPlot(label, times, points.row(1), "r");
  if(plot_reference)
  {
    plt::labelPlot("reference", times, ref_points.row(1), "b");
  }
  plt::subplot(3, 1, 3);
  plt::labelPlot(label, times, points.row(2), "r");
  if(plot_reference)
  {
    plt::labelPlot("reference", times, ref_points.row(2), "b");
  }
  plt::show(false);
}

//-----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::plotOrientationError(
    const std::vector<FloatType>& times,
    const std::vector<Matrix3>& est,
    const std::string& label)
{
  CHECK_EQ(times.size(), est.size());

  Eigen::Matrix<FloatType, 1, Eigen::Dynamic> err(1, est.size());

  for (size_t i = 0; i < est.size(); ++i)
  {
    Quaternion q1(est[i]);
    Quaternion q2(trajectory_->orientation(times[i]));
    err(i) = q1.getDisparityAngle(q2);
  }
  plt::figure("orientation_offset");
  plt::title("Orientation Offset");
  plt::labelPlot(label, times, err.row(0));
  plt::legend();

  plt::show(false);
}

//-----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::plotImuMeasurements(
    const std::vector<FloatType>& times,
    const ImuAccGyrContainer& measurements1,
    const ImuAccGyrContainer& measurements2)
{
  CHECK_EQ(static_cast<int>(times.size()), measurements1.size());
  CHECK_EQ(static_cast<int>(times.size()), measurements2.size());
  plt::figure();
  plt::subplot(3, 1, 1);
  plt::title("Imu Measurements (Accel)");
  plt::plot(times, measurements1.row(0));
  plt::plot(times, measurements2.row(0));

  plt::subplot(3, 1, 2);
  plt::plot(times, measurements1.row(1));
  plt::plot(times, measurements2.row(1));

  plt::subplot(3, 1, 3);
  plt::plot(times, measurements1.row(2));
  plt::plot(times, measurements2.row(2));

  plt::show(false);

  plt::figure();
  plt::subplot(3, 1, 1);
  plt::title("Imu Measurements (Gyro)");
  plt::plot(times, measurements1.row(3));
  plt::plot(times, measurements2.row(3));

  plt::subplot(3, 1, 2);
  plt::plot(times, measurements1.row(4));
  plt::plot(times, measurements2.row(4));

  plt::subplot(3, 1, 3);
  plt::plot(times, measurements1.row(5));
  plt::plot(times, measurements2.row(5));

  plt::show(false);

  VLOG(1) << "Measurements: " << measurements1.size() << "\n";
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

  VLOG(1) << "Finish Processing.";
  node.shutdown();
  VLOG(1) << "Node terminated cleanly.";
  return 0;
}
