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
      const Matrix3& gyroscope_noise_covariance,
      const std::vector<Matrix3>* D_R_i_k_reference,
      FloatType start,
      FloatType end);

  PreIntegratorMonteCarlo::Ptr runManifoldCorrupted(
      PreIntegrationRunner::Ptr preintegration_runner,
      const Matrix3& gyroscope_noise_covariance,
      FloatType start,
      FloatType end);

  //! Given the GFLags configuration loads and returns a bspline representing
  //! a trajectory.
  void loadTrajectory();

  //! Show the trajectory that was loaded or generated.
  void showTrajectory();

  //! Generate a series of plots that show the results
  void plotCovarianceResults(std::initializer_list<const std::vector<Matrix3>>
                             covariances_vectors);

  //! Generate a series of plots that show the results
  void plotCovarianceOffsets(const std::vector<Matrix3> ref,
                             const std::vector<Matrix3> est);

  //! Plot the estimated / integrated vector of rotation matrices.
  void plotOrientation(const std::vector<FloatType>& times,
                       const std::vector<Matrix3>& R_i);

  //! Plot the measurements used to obtain a preintegration state.
  void plotImuMeasurements(const std::vector<FloatType>& times,
                           const ImuAccGyr& measurements,
                           const ImuAccGyr& measurements2);

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
  GaussianSampler<3>::Ptr accel_noise = GaussianSampler<3>::variances(accel_covar);
  GaussianSampler<3>::Ptr gyro_noise = GaussianSampler<3>::variances(gyro_covar);

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

  PreIntegratorMonteCarlo::Ptr mc_corrupted =
      runManifoldCorrupted(preintegration_runner,
                           gyroscope_noise_covariance,
                           start,
                           end);

  // Use the corrupted MC simulator to get an unperturbed orientation estimate.
  PreIntegrator::Ptr actual_integrator = mc_corrupted->preintegrateActual(start,
                                                                          end);
  runManifoldClean(preintegration_runner,
                   gyroscope_noise_covariance,
                   &actual_integrator->D_R_i_k(),
                   start,
                   end);

//  plotOrientation(est_integrator->times_raw(), est_integrator->R_i_k());
//  plotImuMeasurements(est_integrator->times_raw(),
//                      est_integrator->measurements(),
//                      actual_integrator->measurements());
}

// -----------------------------------------------------------------------------
PreIntegratorMonteCarlo::Ptr PreIntegrationEvaluationNode::runManifoldClean(
    PreIntegrationRunner::Ptr preintegration_runner,
    const Matrix3& gyroscope_noise_covariance,
    const std::vector<Matrix3>* D_R_i_k_reference,
    FloatType start,
    FloatType end)
{
  VLOG(1) << "Monte Carlo Simulation [ManifoldPreIntegrator:Clean]";
  PreIntegratorFactory::Ptr preintegrator_factory_clean(
        std::make_shared<ManifoldPreIntegrationFactory>(gyroscope_noise_covariance,
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
                         est_integrator->covariance_i_k()});

  plotCovarianceOffsets(mc->covariances(),
                        est_integrator->covariance_i_k());

  return mc;
}

// -----------------------------------------------------------------------------
PreIntegratorMonteCarlo::Ptr PreIntegrationEvaluationNode::runManifoldCorrupted(
    PreIntegrationRunner::Ptr preintegration_runner,
    const Matrix3& gyroscope_noise_covariance,
    FloatType start,
    FloatType end)
{
  VLOG(1) << "Monte Carlo Simulation [ManifoldPreIntegrator:Corrupted]";
  PreIntegratorFactory::Ptr preintegrator_factory(
        std::make_shared<ManifoldPreIntegrationFactory>(gyroscope_noise_covariance));
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
                         est_integrator->covariance_i_k()});

  plotCovarianceOffsets(mc->covariances(),
                        est_integrator->covariance_i_k());

  return mc;
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
    std::initializer_list<const std::vector<Matrix3>> covariances_vectors)
{

  plt::figure("covariances");
  for(auto elem: covariances_vectors)
  {
    Eigen::Matrix<FloatType, 3, Eigen::Dynamic> v(3, elem.size());
    for (size_t i = 0; i < elem.size(); ++i)
    {
      v.col(i) = elem[i].diagonal();
    }
    plt::subplot(3, 1, 1);
    plt::plot(v.row(0));
    plt::subplot(3, 1, 2);
    plt::plot(v.row(1));
    plt::subplot(3, 1, 3);
    plt::plot(v.row(2));
  }
  plt::show(false);
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::plotCovarianceOffsets(
    const std::vector<Matrix3> ref,
    const std::vector<Matrix3> est)
{
  CHECK_EQ(ref.size(), est.size());

  plt::figure("covariance_offsets");
  Eigen::Matrix<FloatType, 3, Eigen::Dynamic> v(3, ref.size());
  for (size_t i = 0; i < ref.size(); ++i)
  {
    v.col(i) = (ref[i].diagonal() - est[i].diagonal()).cwiseAbs();
  }
  plt::subplot(3, 1, 1);
  plt::plot(v.row(0));
  plt::subplot(3, 1, 2);
  plt::plot(v.row(1));
  plt::subplot(3, 1, 3);
  plt::plot(v.row(2));

  plt::show(false);
}

//-----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::plotOrientation(
    const std::vector<FloatType>& times,
    const std::vector<Matrix3>& orientation)
{
  Eigen::Matrix<FloatType, 3, Eigen::Dynamic> points(3, orientation.size());
  Eigen::Matrix<FloatType, 3, Eigen::Dynamic> ref_points(3, orientation.size());

  for (size_t i = 0; i < orientation.size() - 1; ++i)
  {
    ze::sm::RotationVector rv(orientation[i]);
    points.col(i) = rv.getParameters();
    ref_points.col(i) = trajectory_->eval(times[i]).tail<3>();
  }
  plt::figure();
  plt::subplot(3, 1, 1);
  plt::title("Orientation");
  plt::plot(points.row(0), "r");
  plt::plot(ref_points.row(0), "b");

  plt::subplot(3, 1, 2);
  plt::plot(points.row(1), "r");
  plt::plot(ref_points.row(1), "b");

  plt::subplot(3, 1, 3);
  plt::plot(points.row(2), "r");
  plt::plot(ref_points.row(2), "b");

  plt::show(false);
}

//-----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::plotImuMeasurements(
    const std::vector<FloatType>& times,
    const ImuAccGyr& measurements1,
    const ImuAccGyr& measurements2)
{
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
