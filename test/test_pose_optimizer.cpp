#include <random>
#include <ze/common/test_entrypoint.h>
#include <ze/common/matrix.h>
#include <ze/common/timer.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/pose_optimizer.h>
#include <ze/geometry/robust_cost.h>

TEST(PoseOptimizerTests, testSolver)
{
  using namespace ze;

  Transformation T_C_B, T_B_W;
  T_C_B.setRandom(); // Random camera to imu/body transformation.
  T_B_W.setRandom(); // Random body transformation.

  const size_t n = 120;
  PinholeCamera cam = createPinholeCamera(640, 480, 329.11, 329.11, 320.0, 240.0);
  Keypoints px_true = generateRandomKeypoints(640, 480, 10, n);

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
  const double stddev = 1.0;
  std::normal_distribution<double> px_noise(0.0, stddev);
  for(size_t i = 0; i < n; ++i)
  {
    px_noisy(0,i) += px_noise(gen);
    px_noisy(1,i) += px_noise(gen);
  }
  Bearings bearings_noisy = cam.backProjectVectorized(px_noisy);

  // Perturb pose:
  Transformation T_B_W_perturbed =
      T_B_W * Transformation::exp((Vector6() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

  // Optimize using bearing vectors:
  PoseOptimizerFrameData data;
  data.f = bearings_noisy;
  data.p_W = pos_W;
  data.T_C_B = T_C_B;

  // Without prior:
  {
    double pos_prior_weight = 0.0;
    double rot_prior_weight = 0.0;
    ze::Timer t;
    PoseOptimizerFrameDataVec data_vec = { data };
    PoseOptimizer optimizer(data_vec, T_B_W, pos_prior_weight, rot_prior_weight);
    Transformation T_B_W_estimate = T_B_W_perturbed;
    optimizer.optimize(T_B_W_estimate);
    VLOG(1) << "optimization took " << t.stopAndGetMilliseconds() << " ms\n";

    // Compute error:
    Transformation T_err = T_B_W * T_B_W_estimate.inverse();
    double pos_error = T_err.getPosition().norm();
    double ang_error = T_err.getRotation().log().norm();
    CHECK_LT(pos_error, 0.005);
    CHECK_LT(ang_error, 0.005);
    VLOG(1) << "ang error = " << ang_error;
    VLOG(1) << "pos error = " << pos_error;
  }

  // With rotation prior:
  {
    double pos_prior_weight = 0.0;
    double rot_prior_weight = 10.0;
    ze::Timer t;
    PoseOptimizerFrameDataVec data_vec = { data };
    PoseOptimizer optimizer(data_vec, T_B_W, pos_prior_weight, rot_prior_weight);
    Transformation T_B_W_estimate = T_B_W_perturbed;
    optimizer.optimize(T_B_W_estimate);
    VLOG(1) << "optimization took " << t.stopAndGetMilliseconds() << " ms\n";

    // Compute error:
    Transformation T_err = T_B_W * T_B_W_estimate.inverse();
    double pos_error = T_err.getPosition().norm();
    double ang_error = T_err.getRotation().log().norm();
    CHECK_LT(pos_error, 0.005);
    CHECK_LT(ang_error, 0.005);
    VLOG(1) << "ang error = " << ang_error;
    VLOG(1) << "pos error = " << pos_error;
  }

  // With position prior:
  {
    double pos_prior_weight = 10.0;
    double rot_prior_weight = 0.0;
    ze::Timer t;
    PoseOptimizerFrameDataVec data_vec = { data };
    PoseOptimizer optimizer(data_vec, T_B_W, pos_prior_weight, rot_prior_weight);
    Transformation T_B_W_estimate = T_B_W_perturbed;
    optimizer.optimize(T_B_W_estimate);
    VLOG(1) << "optimization took " << t.stopAndGetMilliseconds() * 1000 << " ms\n";

    // Compute error:
    Transformation T_err = T_B_W * T_B_W_estimate.inverse();
    double pos_error = T_err.getPosition().norm();
    double ang_error = T_err.getRotation().log().norm();
    CHECK_LT(pos_error, 0.005);
    CHECK_LT(ang_error, 0.005);
    VLOG(1) << "ang error = " << ang_error;
    VLOG(1) << "pos error = " << pos_error;
  }
}


ZE_UNITTEST_ENTRYPOINT
