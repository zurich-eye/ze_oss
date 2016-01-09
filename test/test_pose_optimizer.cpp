#include <random>
#include <ze/common/test/entrypoint.h>
#include <ze/common/timer.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/pose_optimizer.h>
#include <ze/geometry/robust_cost.h>

TEST(NllsPoseOptimizerTests, testSolver)
{
  using namespace ze;

  Transformation T_C_B, T_B_W;
  T_C_B.setRandom(); // Random camera to imu/body transformation.
  T_B_W.setRandom(); // Random body transformation.

  const size_t n = 200;
  PinholeCamera cam(640, 480, 329.11, 329.11, 320.0, 240.0);
  Keypoints pix_true = generateRandomKeypoints(640, 480, 10, n);

  Positions pos_C = cam.backProjectVectorized(pix_true);

  // Obtain the 3D points by applying a random scaling between 1 and 3 meters.
  std::ranlux24 gen;
  std::uniform_real_distribution<double> scale(1.0, 3.0);
  for(size_t i = 0; i < n; ++i) {
    pos_C.col(i) *= scale(gen);
  }

  // Transform points to world coordinates.
  Positions pos_W = (T_B_W.inverse() * T_C_B.inverse()).transformVectorized(pos_C);

  // Apply some noise to the keypoints to simulate measurements.
  Keypoints pix_noisy = pix_true;
  const double stddev = 1.0;
  std::normal_distribution<double> px_noise(0.0, stddev);
  for(size_t i = 0; i < n; ++i) {
    pix_noisy(0,i) += px_noise(gen);
    pix_noisy(1,i) += px_noise(gen);
  }

  // Perturb pose:
  ze::Timer t;
  Transformation T_B_W_perturbed =
      T_B_W * Transformation::exp((Eigen::Matrix<double, 6, 1>() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
  PoseOptimizer optimizer(pix_noisy, pos_W, T_C_B, cam, 1.0);
  optimizer.optimize(T_B_W_perturbed);
  std::cout << "optimization took " << t.stop() * 1000 << " ms\n";
  Transformation T_err = T_B_W * T_B_W_perturbed.inverse();
  std::cout << T_err << std::endl;

}


ZE_UNITTEST_ENTRYPOINT
