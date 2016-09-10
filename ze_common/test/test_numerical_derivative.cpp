// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <cmath>

#include <ze/common/types.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/numerical_derivative.hpp>

TEST(NumericalDerivativeTests, testLinearVector)
{
#ifndef ZE_SINGLE_PRECISION_FLOAT
  using namespace ze;

  Matrix2 J = numericalDerivative<Vector2, Vector2>(
        [](const Vector2& x) { return x * 1.2; }, Vector2(1, 2));
  EXPECT_NEAR(J(0,0), 1.2, 1e-8);
  EXPECT_NEAR(J(1,1), 1.2, 1e-8);
  EXPECT_NEAR(J(0,1), 0.0, 1e-8);
  EXPECT_NEAR(J(1,0), 0.0, 1e-8);
#else
  LOG(WARNING) << "Test ignored for single precision float.";
#endif
}

ZE_UNITTEST_ENTRYPOINT
