#include <random>
#include <ze/common/benchmark.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/line.hpp>
#include <ze/common/matrix.h>
#include <ze/common/timer.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/pose_optimizer.h>
#include <ze/geometry/robust_cost.h>

namespace ze {

void testPoseOptimizer(
    const FloatType pos_prior_weight,
    const FloatType rot_prior_weight,
    const Transformation& T_B_W,
    const Transformation& T_B_W_perturbed,
    const PoseOptimizerFrameData& data,
    const std::string& description)
{
  PoseOptimizerFrameDataVec data_vec = { data };
  Transformation T_B_W_estimate;
  auto fun = [&]()
  {
    PoseOptimizer optimizer(
          PoseOptimizer::getDefaultSolverOptions(),
          data_vec, T_B_W, pos_prior_weight, rot_prior_weight);
    T_B_W_estimate = T_B_W_perturbed;
    optimizer.optimize(T_B_W_estimate);
  };
  runTimingBenchmark(fun, 1, 10, description, true);

  // Compute error:
  Transformation T_err = T_B_W * T_B_W_estimate.inverse();
  FloatType pos_error = T_err.getPosition().norm();
  FloatType ang_error = T_err.getRotation().log().norm();
  EXPECT_LT(pos_error, 0.005);
  EXPECT_LT(ang_error, 0.005);
  VLOG(1) << "ang error = " << ang_error;
  VLOG(1) << "pos error = " << pos_error;
}

} // namespace ze

TEST(PoseOptimizerTests, testSolver)
{
  using namespace ze;

  Transformation T_C_B, T_B_W;
  T_C_B.setRandom(); // Random camera to imu/body transformation.
  T_B_W.setRandom(); // Random body transformation.

  const size_t n = 120;
  PinholeCamera cam = createTestCamera();
  Keypoints px_true = generateRandomKeypoints(cam.size(), 10, n);

  Positions pos_C = cam.backProjectVectorized(px_true);

  // Obtain the 3D points by applying a random scaling between 1 and 3 meters.
  std::ranlux24 gen;
  std::uniform_real_distribution<double> scale(1.0, 3.0);
  for(size_t i = 0; i < n; ++i)
  {
    pos_C.col(i) *= scale(gen);
  }

  // Transform points to world coordinates.
  Positions pos_W = (T_B_W.inverse() * T_C_B.inverse()).transformVectorized(pos_C);

  // Apply some noise to the keypoints to simulate measurements.
  Keypoints px_noisy = px_true;
  VectorX pyr_scale(n);
  const double stddev = 1.0;
  std::normal_distribution<double> px_noise(0.0, stddev);
  for(size_t i = 0; i < n; ++i)
  {
    // Features distribute among all levels. features on higher levels have more
    // uncertainty.
    pyr_scale(i) = (1 << (i % 4));
    px_noisy(0,i) += pyr_scale(i) * px_noise(gen);
    px_noisy(1,i) += pyr_scale(i) * px_noise(gen);
  }
  Bearings bearings_noisy = cam.backProjectVectorized(px_noisy);

  // Perturb pose:
  Transformation T_B_W_perturbed =
      T_B_W * Transformation::exp((Vector6() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

  // Optimize using bearing vectors:
  PoseOptimizerFrameData data;
  data.f = bearings_noisy;
  data.kp_idx = KeypointIndices(n, 1);
  data.p_W = pos_W;
  data.T_C_B = T_C_B;
  data.scale = pyr_scale;
  data.type = PoseOptimizerResidualType::UnitPlane;

  testPoseOptimizer(
        0.0, 0.0, T_B_W, T_B_W_perturbed, data, "UnitPlane, No Prior");
  testPoseOptimizer(
        10.0, 0.0, T_B_W, T_B_W_perturbed, data, "UnitPlane, Rotation Prior");
  testPoseOptimizer(
        10.0, 10.0, T_B_W, T_B_W_perturbed, data, "UnitPlane, Rotation and Position Prior");

  data.type = PoseOptimizerResidualType::Bearing;

  testPoseOptimizer(
        0.0, 0.0, T_B_W, T_B_W_perturbed, data, "Bearing, No Prior");
  testPoseOptimizer(
        10.0, 0.0, T_B_W, T_B_W_perturbed, data, "Bearing, Rotation Prior");
  testPoseOptimizer(
        10.0, 10.0, T_B_W, T_B_W_perturbed, data, "Bearing, Rotation and Position Prior");
}

TEST(PoseOptimizerTests, testSolver_withLines)
{
  using namespace ze;

  Transformation T_C_B, T_B_W;
  T_C_B.setRandom(); // Random camera to imu/body transformation.
  T_B_W.setRandom(); // Random body transformation.

  const size_t n = 120;
  PinholeCamera cam = createTestCamera();
  Keypoints endpoints_image = generateRandomKeypoints(cam.size(), 10, 2 * n);
  Positions endpoints_C = cam.backProjectVectorized(endpoints_image);
  // Obtain the 3D points by applying a random scaling between 1 and 3 meters.
  std::ranlux24 gen;
  std::uniform_real_distribution<double> scale(1.0, 3.0);
  for(size_t i = 0; i < 2 * n; ++i)
  {
    endpoints_C.col(i) *= scale(gen);
  }
  Positions endpoints_W =
      (T_B_W.inverse() * T_C_B.inverse()).transformVectorized(endpoints_C);

  Lines lines_W;
  generateLinesFromEndpoints(endpoints_W.block(0, 0, 3, n),
                             endpoints_W.block(0, n, 3, n),
                             lines_W);

  // Apply some noise to the endpoints to simulate measurements.
  Keypoints endpoints_noisy = endpoints_image;
  const double stddev = 1.0;
  std::normal_distribution<double> endpoints_noise(0.0, stddev);
  for (size_t i = 0; i < 2 * n; ++i)
  {
    endpoints_noisy(0, i) += endpoints_noise(gen);
    endpoints_noisy(1, i) += endpoints_noise(gen);
  }
  Bearings bearings_noisy = cam.backProjectVectorized(endpoints_noisy);
  Bearings bearings_truth = cam.backProjectVectorized(endpoints_image);
  LineMeasurements line_measurements_noisy(3, n);
  LineMeasurements line_measurements_truth(3, n);
  for (size_t i = 0; i < n; ++i)
  {
    line_measurements_noisy.col(i) =
        (bearings_noisy.col(i).cross(bearings_noisy.col(n + i))).normalized();
    // Since b1xb2 = -(b2xb1) we choose the normal with positive first entry.
    if (line_measurements_noisy(0, i) < 0.0)
    {
      line_measurements_noisy.col(i) *= -1;
    }

    line_measurements_truth.col(i) =
        (bearings_truth.col(i).cross(bearings_truth.col(n + i))).normalized();
    if (line_measurements_truth(0, i) < 0.0)
    {
      line_measurements_truth.col(i) *= -1;
    }
  }

  // Check if error for truth is zero.
  PoseOptimizerFrameData data;
  data.line_measurements_C = line_measurements_truth;
  data.lines_W = lines_W;
  data.T_C_B = T_C_B;
  data.type = PoseOptimizerResidualType::Line;

  PoseOptimizerFrameDataVec data_vec = { data };
  PoseOptimizer optimizer(
        PoseOptimizer::getDefaultSolverOptions(),
        data_vec, T_B_W, 0.0, 0.0);
  FloatType error = optimizer.evaluateError(T_B_W, nullptr, nullptr);
  EXPECT_NEAR(error, 0.0, 1e-5);

  // Perturb pose:
  Transformation T_B_W_perturbed =
      T_B_W * Transformation::exp((Vector6() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

  // Optimize using noisy lines:
  data.line_measurements_C = line_measurements_noisy;
  testPoseOptimizer(
        0.0, 0.0, T_B_W, T_B_W_perturbed, data, "Line, No Prior");
}

ZE_UNITTEST_ENTRYPOINT
