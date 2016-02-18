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
  using namespace ze;

  Transformation T_A_B, T_W_A, T_W_B;
  T_A_B.setRandom(1.0);
  T_W_A.setRandom(1.0);
  T_W_B.setRandom(1.0);

  auto residualLambda = [&](const Transformation& T_A_B) {
      Vector3 pos = T_W_A * T_A_B.getPosition() - T_W_B.getPosition();
      Vector3 rot = Quaternion::log(T_W_B.getRotation().inverse()
                                    * (T_W_A.getRotation() * T_A_B.getRotation()));
      return (Vector6() << pos, rot).finished();
    };
  Matrix6 J_numeric = numericalDerivative<Vector6, Transformation>(residualLambda, T_A_B);
  Matrix6 J_analytic = dRelpose_dTransformation(T_A_B, T_W_A, T_W_B);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(J_numeric, J_analytic));
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
  Transformation T_A_B;
  T_A_B.setRandom();

  // Compute transformed trajectory.
  TransformationVector T_W_B(n_poses);
  for (size_t i = 0; i < n_poses; ++i)
  {
    T_W_B.at(i) = T_W_A.at(i) * T_A_B;
  }

  // Perturb estimated pose.
  FloatType sigma_pos = 0.2;
  FloatType sigma_rot = 30.0 / 180 * M_PI;
  Vector3 pos_pert = Vector3::Random();
  pos_pert = pos_pert.normalized() * sigma_pos;
  Vector3 rot_pert = Vector3::Random();
  rot_pert = rot_pert.normalized() * sigma_rot;
  Transformation T_pert = Transformation::exp((Vector6() << pos_pert, rot_pert).finished());
  Transformation T_A_B_estimate = T_A_B * T_pert;

  // Optimize.
  PoseAligner problem(T_W_A, T_W_B, sigma_pos, sigma_rot);
  problem.optimize(T_A_B_estimate);

  // Compute error.
  Transformation T_err = T_A_B.inverse() * T_A_B_estimate;
  EXPECT_LT(T_err.log().norm(), 1e-10);
}

ZE_UNITTEST_ENTRYPOINT
