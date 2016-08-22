#include <cmath>
#include <functional>
#include <random>
#include <utility>

#include <ze/common/numerical_derivative.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/transformation.h>
#include <ze/common/types.h>
#include <ze/geometry/align_poses.h>

TEST(AlignPosesTest, testJacobian)
{
#ifndef ZE_SINGLE_PRECISION_FLOAT
  using namespace ze;

  Transformation T_A0_B0, T_Ai_A0, T_B0_Bi;
  T_A0_B0.setRandom(1.0);
  T_Ai_A0.setRandom(1.0);
  T_B0_Bi.setRandom(1.0);

  auto residualLambda = [&](const Transformation& T_A0_B0) {
      return (T_Ai_A0 * T_A0_B0 * T_B0_Bi).log();
    };
  Matrix6 J_numeric = numericalDerivative<Vector6, Transformation>(residualLambda, T_A0_B0);
  Matrix6 J_analytic = dRelpose_dTransformation(T_A0_B0, T_Ai_A0, T_B0_Bi);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(J_numeric, J_analytic));
#else
  LOG(WARNING) << "Numerical derivative test ignored for single precision float.";
#endif
}

TEST(AlignPosesTest, testOptimization)
{
  using namespace ze;

  const size_t n_poses = 20;

  // Generate random trajectory.
  TransformationVector T_W_A(n_poses);
  for (Transformation& T : T_W_A)
  {
    T.setRandom(2.0);
  }

  // Random transformation between trajectories.
  Transformation T_A0_B0;
  T_A0_B0.setRandom();

  // Compute transformed trajectory.
  TransformationVector T_W_B(n_poses);
  for (size_t i = 0; i < n_poses; ++i)
  {
    Transformation T_A0_Ai = T_W_A[0].inverse() * T_W_A[i];
    T_W_B[i] = T_W_A[0] * T_A0_B0 * T_A0_Ai;
  }

  // Perturb estimated pose.
  real_t sigma_pos = 0.05;
  real_t sigma_rot = 5.0 / 180 * M_PI;
  Vector3 pos_pert = Vector3::Random();
  pos_pert = pos_pert.normalized() * sigma_pos;
  Vector3 rot_pert = Vector3::Random();
  rot_pert = rot_pert.normalized() * sigma_rot;
  Transformation T_pert = Transformation::exp((Vector6() << pos_pert, rot_pert).finished());
  Transformation T_A0_B0_estimate = T_A0_B0 * T_pert;

  // Optimize.
  PoseAligner problem(T_W_A, T_W_B, sigma_pos, sigma_rot);
  problem.optimize(T_A0_B0_estimate);

  // Compute error.
  Transformation T_err = T_A0_B0.inverse() * T_A0_B0_estimate;
  EXPECT_LT(T_err.log().norm(), 1.5e-5);
}

ZE_UNITTEST_ENTRYPOINT
