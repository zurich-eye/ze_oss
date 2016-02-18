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
      return p_A - T_A_B * p_B;
    };
  Matrix36 J_numeric = numericalDerivative<Vector3, Transformation>(residualLambda, T_A_B);
  Matrix36 J_analytic = dPointdistance_dRelpose(T_A_B, p_A, p_B);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(J_numeric, J_analytic));
}

ZE_UNITTEST_ENTRYPOINT
