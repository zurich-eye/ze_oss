#include <functional>
#include <memory>

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
  int trajectory_spline_order = 3;

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

  //! Given the GFLags configuration loads and returns a bspline representing
  //! a trajectory.
  void loadTrajectory();

  //! Show the trajectory that was loaded or generated.
  void showTrajectory();

  //! Generate a series of plots that show the results
  void plotResults(const std::vector<Matrix3>& covariances_mc,
                   const std::vector<Matrix3>& covariances_est);

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
  visualizer_ = std::make_shared<VisualizerRos>();
  splines_visualizer_ = std::make_shared<SplinesVisualizer>(visualizer_);

  parameters_ = ImuPreIntegrationParameters::fromGFlags();

  Vector3 accel_covar = parameters_.accel_noise_density
                        * parameters_.accel_noise_density
                        * Vector3::Identity();
  Vector3 gyro_covar = parameters_.gyro_noise_density
                       * parameters_.gyro_noise_density
                       * Vector3::Identity();
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

  PreIntegrationRunner::Ptr preintegraton_runner =
      std::make_shared<PreIntegrationRunner>(
        scenario_runner,
        parameters_.imu_sampling_time,
        parameters_.camera_sampling_time);

  VLOG(1) << "Initialize monte carlo runner";
  PreIntegratorMonteCarlo<ManifoldPreIntegrationState> mc(preintegraton_runner,
                                                          gyroscope_noise_covariance);

  VLOG(1) << "Monte Carlo Simulation";
  mc.simulate(FLAGS_monte_carlo_runs, scenario->start(), scenario->end());

  VLOG(1) << "Reference Estimate";
  ManifoldPreIntegrationState::Ptr est_integrator = mc.preintegrate_corrupted(
                                                      start, end);

  plotResults(mc.covariances(), est_integrator->covariances());

  plotResults(mc.covariances_absolute(), est_integrator->covariances());
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
          Vector3::Identity() * parameters_.imu_acc_bias_noise_density,
          Vector3::Identity() * parameters_.imu_gyr_bias_noise_density,
          start,
          end,
          1000); // This is an arbitrary value.
  }
  //! a simple constant bias
  else
  {
    Vector3 accel_bias = Vector3::Identity() * parameters_.imu_acc_bias_const;
    Vector3 gyro_bias = Vector3::Identity() * parameters_.imu_gyr_bias_const;

    return std::make_shared<ConstantBias>(accel_bias, gyro_bias);
  }
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::showTrajectory()
{
  splines_visualizer_->plotSpline(*trajectory_);
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::plotResults(
    const std::vector<Matrix3>& covariances_mc,
    const std::vector<Matrix3>& covariances_est)
{
  Eigen::Matrix<FloatType, 3, Eigen::Dynamic> mc(3, covariances_mc.size());
  Eigen::Matrix<FloatType, 3, Eigen::Dynamic> est(3, covariances_est.size());

  for (size_t i = 0; i < covariances_mc.size(); ++i)
  {
    mc.col(i) = covariances_mc[i].diagonal();
    est.col(i) = covariances_est[i].diagonal();
  }

  plt::subplot(3, 1, 1);
  plt::plot(mc.row(0), "r");
  plt::plot(est.row(0), "b");

  plt::subplot(3, 1, 2);
  plt::plot(mc.row(1), "r");
  plt::plot(est.row(1), "b");

  plt::subplot(3, 1, 3);
  plt::plot(mc.row(2), "r");
  plt::plot(est.row(2), "b");

  plt::show();
}

// -----------------------------------------------------------------------------
void PreIntegrationEvaluationNode::shutdown()
{
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
