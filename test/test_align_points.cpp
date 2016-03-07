#include <cmath>
#include <functional>
#include <random>
#include <utility>

#include <ze/common/numerical_derivative.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/transformation.h>
#include <ze/common/types.h>
#include <ze/geometry/align_points.h>

TEST(AlignPointsTest, testJacobian)
{
  using namespace ze;

  Vector3 p_A = Vector3::Random();
  Vector3 p_B = Vector3::Random();
  Transformation T_A_B;
  T_A_B.setRandom(1.0);

  auto residualLambda = [&](const Transformation& T_A_B) {
      return Vector3(p_A - T_A_B * p_B);
    };

  Matrix36 J_numeric = numericalDerivative<Vector3, Transformation>(residualLambda, T_A_B);
  Matrix36 J_analytic = dPointdistance_dRelpose(T_A_B, p_A, p_B);

  std::cout << "J_numeric: \n" << J_numeric << std::endl;

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(J_numeric, J_analytic));
}

TEST(AlignPosesTest, testOptimization)
{
  using namespace ze;

  const size_t n_points = 100;

  // Generate random points.
  Positions p_B(3, n_points);
  for (size_t i = 0; i < n_points; ++i)
  {
    p_B.col(i) = Vector3::Random();
  }

  // Random transformation between trajectories.
  Transformation T_A_B;
  T_A_B.setRandom();

  // Compute transformed points.
  Positions p_A = T_A_B.transformVectorized(p_B);

  // Align trajectories.
  PointAligner problem(p_A, p_B);
  Vector6 perturbation = Vector6::Ones() * 0.1;
  Transformation T_A_B_estimate = T_A_B * Transformation::exp(perturbation);
  problem.optimize(T_A_B_estimate);

  // Compute error.
  Transformation T_err = T_A_B.inverse() * T_A_B_estimate;
  EXPECT_LT(T_err.log().norm(), 1e-10);
}

ZE_UNITTEST_ENTRYPOINT
