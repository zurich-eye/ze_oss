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

DEFINE_string(curve_source, "", "Path to file to load curve from, default: generate random curve");

namespace ze {

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
  BSplinePoseMinimalRotationVector getBSpline();

  void shutdown();

private:

};

// -----------------------------------------------------------------------------
PreIntegrationEvaluationNode::PreIntegrationEvaluationNode()
{
  FloatType imu_sampling_time; // FLAGS
  FloatType camera_sampling_time; // FLAGS
  FloatType accel_noise_bandwidth_hz; // FLAGS
  FloatType gyro_nois_bandwidth_hz; // FLAGS
  Vector3 gravity(0, 0, -9.81);
  Vector3 accel_covar; // FLAGS
  Vector3 gyro_covar; // FLAGS
  Matrix3 gyroscope_noise_covariance; // FLAGS

  BSplinePoseMinimalRotationVector bspline = getBSpline();

  FloatType start = bspline.t_min();
  FloatType end = bspline.t_max();

  Scenario::Ptr scenario = std::make_shared<SplineScenario>(bspline);

  GaussianSampler<3>::Ptr accel_noise = GaussianSampler<3>::sigmas(accel_covar);
  GaussianSampler<3>::Ptr gyro_noise = GaussianSampler<3>::sigmas(gyro_covar);

  ScenarioRunner::Ptr scenario_runner =
      std::make_shared<ScenarioRunner>(scenario,
                                       imuBias(start, end),
                                       accel_noise,
                                       gyro_noise,
                                       accel_noise_bandwidth_hz,
                                       gyro_nois_bandwidth_hz,
                                       gravity);

  PreIntegrationRunner::Ptr preintegraton_runner =
      std::make_shared<PreIntegrationRunner>(
        scenario_runner, imu_sampling_time, camera_sampling_time);

  PreIntegratorMonteCarlo<ManifoldPreIntegrationState> mc(preintegraton_runner,
                                                          gyroscope_noise_covariance);

  mc.simulate(100, scenario->start(), scenario->end());
}

BSplinePoseMinimalRotationVector PreIntegrationEvaluationNode::getBSpline()
{
  // A bspline fixed at 3rd order.
  BSplinePoseMinimalRotationVector bspline(3);
  // generate random
  if (FLAGS_curve_source == "")
  {
    FloatType start; // FLAGS
    FloatType end; // FLAGS
    size_t num_interpolation_points; // FLAGS
    size_t num_segments; // FLAGS
    FloatType lambda; // FLAGS

    MatrixX points(6, num_interpolation_points);
    points.setRandom();
    // make translations significanter
    points.block(0, 0, 3, num_interpolation_points) *= 10;

    VectorX times;
    times.setLinSpaced(num_interpolation_points, start, end);

    bspline.initPoseSpline3(times, points, num_segments, lambda);
  }
  // load from file
  else
  {
    CHECK_EQ(false, true) << "Not implemented.";
  }

  return bspline;
}

ImuBias::Ptr PreIntegrationEvaluationNode::imuBias(FloatType start,
                                                   FloatType end)
{
  std::string imu_bias_type; // FLAGS

  //! continuous bias model
  if (imu_bias_type == "continuous")
  {
    FloatType acc_bias_noise_density; // FLAGS
    FloatType gyr_bias_noise_density; // FLAGS

    return std::make_shared<ContinuousBias>(
          Vector3::Identity() * acc_bias_noise_density,
          Vector3::Identity() * gyr_bias_noise_density,
          start,
          end,
          1000); // This is an arbitrary value.
  }
  //! a simple constant bias
  else
  {
    Vector3 accel_bias; // FLAGS
    Vector3 gyro_bias; // FLAGS

    return std::make_shared<ConstantBias>(accel_bias, gyro_bias);
  }
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
