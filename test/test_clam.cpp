#include <random>
#include <ze/common/test_entrypoint.h>
#include <ze/common/matrix.h>
#include <ze/common/numerical_derivative.h>
#include <ze/common/timer.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/clam.h>

TEST(ClamTests, testJacobians)
{
  using namespace ze;

  Transformation T_C_B, T_Bc_Br;
  T_C_B.setRandom(); // Random camera to imu/body transformation.
  T_Bc_Br = Transformation::exp((Vector6() << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1).finished());

  PinholeCamera cam = createPinholeCamera(640, 480, 329.11, 329.11, 320.0, 240.0);
  Keypoint px_Cr(230.4, 325.6);

  Bearing f_Cr = cam.backProject(px_Cr);
  Bearing f_Br = T_C_B.getRotation().inverse().rotate(f_Cr);
  Bearing p_Br = T_C_B.inverse().getPosition();

  FloatType inv_depth = 0.455;

  Keypoint px_Cc = cam.project(T_C_B * T_Bc_Br * T_C_B.inverse() * (f_Cr * (1.0 / inv_depth)));

  Matrix26 H1;
  Matrix21 H2;
  Vector2 res = reprojectionResidual(
        f_Br, p_Br, cam, T_C_B, T_Bc_Br, inv_depth, px_Cc, &H1, &H2);
  EXPECT_TRUE(EIGEN_MATRIX_ZERO(res, 1e-10));

  Matrix26 H1_numeric = numericalDerivative<Vector2, Transformation>(
        std::bind(&reprojectionResidual, f_Br, p_Br, cam, T_C_B,
                  std::placeholders::_1, inv_depth, px_Cc, nullptr, nullptr), T_Bc_Br);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H1, H1_numeric, 1e-6));

  Matrix21 H2_numeric = numericalDerivative<Vector2, FloatType>(
        std::bind(&reprojectionResidual, f_Br, p_Br, cam, T_C_B,
                  T_Bc_Br, std::placeholders::_1, px_Cc, nullptr, nullptr), inv_depth);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H2, H2_numeric, 1e-6));

}


ZE_UNITTEST_ENTRYPOINT
