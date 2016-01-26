#include <cmath>

#include <ze/common/test_entrypoint.h>
#include <ze/common/numerical_derivative.h>

TEST(NumericalDerivativeTests, testLinearVector)
{
  Eigen::Matrix2d J =
      ze::numericalDerivative<Eigen::Vector2d, Eigen::Vector2d>(
        [](const Eigen::Vector2d& x) { return x * 1.2; }, Eigen::Vector2d(1, 2));
  EXPECT_NEAR(J(0,0), 1.2, 1e-8);
  EXPECT_NEAR(J(1,1), 1.2, 1e-8);
  EXPECT_NEAR(J(0,1), 0.0, 1e-8);
  EXPECT_NEAR(J(1,0), 0.0, 1e-8);
}

ZE_UNITTEST_ENTRYPOINT
