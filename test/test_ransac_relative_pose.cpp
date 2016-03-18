#include <random>
#include <ze/common/test_entrypoint.h>
#include <ze/common/matrix.h>
#include <ze/common/timer.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/ransac_relative_pose.h>

TEST(RansacRelativePoseTests, testFivePoint)
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
}


ZE_UNITTEST_ENTRYPOINT
