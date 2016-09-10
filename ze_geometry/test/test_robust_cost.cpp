// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <cmath>
#include <random>
#include <utility>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/types.hpp>
#include <ze/geometry/robust_cost.hpp>

TEST(RobustCostTest, testScaleEstimators)
{
  using namespace ze;

  // Generate normally distributed errors with standard deviation 3.0
  std::ranlux24 gen;
  std::normal_distribution<real_t> noise(0.0, 3.0);
  constexpr int n = 1000;
  VectorX errors(n);
  errors.setZero();
  for(int i = 0; i < n; ++i)
    errors(i) = noise(gen);

  double s1 = UnitScaleEstimator<real_t>::compute(errors);
  EXPECT_FLOATTYPE_EQ(s1, 1.0);

  double s2 = NormalDistributionScaleEstimator<real_t>::compute(errors);
  EXPECT_TRUE(std::abs(s2 - 3.0) < 0.2);

  double s3 = MADScaleEstimator<real_t>::compute(errors);
  EXPECT_TRUE(std::abs(s3 - 3.0) < 0.2);
}

TEST(RobustCostTest, testWeightFunctions)
{
  using namespace ze;

  // Generate normally distributed errors with standard deviation 3.0
  std::ranlux24 gen;
  std::normal_distribution<real_t> noise(0.0, 3.0);
  constexpr int n = 10;
  VectorX errors(n);
  errors.setZero();
  for (int i = 0; i < n; ++i)
  {
    errors(i) = noise(gen);
  }

  VectorX errors_scaled = HuberWeightFunction<real_t>::weightVectorized(errors);

  //! @todo: add checks.
  VLOG(1) << errors.transpose();
  VLOG(1) << errors_scaled.transpose();
}

ZE_UNITTEST_ENTRYPOINT
