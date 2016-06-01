#include <cmath>
#include <random>
#include <utility>
#include <ze/common/test_entrypoint.h>
#include <ze/common/types.h>
#include <ze/geometry/robust_cost.h>

TEST(RobustCostTest, testScaleEstimators)
{
  using namespace ze;

  // Generate normally distributed errors with standard deviation 3.0
  std::ranlux24 gen;
  std::normal_distribution<FloatType> noise(0.0, 3.0);
  constexpr int n = 1000;
  VectorX errors(n);
  errors.setZero();
  for(int i = 0; i < n; ++i)
    errors(i) = noise(gen);

  double s1 = UnitScaleEstimator<FloatType>::compute(errors);
  EXPECT_FLOATTYPE_EQ(s1, 1.0);

  double s2 = NormalDistributionScaleEstimator<FloatType>::compute(errors);
  EXPECT_TRUE(std::abs(s2 - 3.0) < 0.2);

  double s3 = MADScaleEstimator<FloatType>::compute(errors);
  EXPECT_TRUE(std::abs(s3 - 3.0) < 0.2);
}

TEST(RobustCostTest, testWeightFunctions)
{
  using namespace ze;

  // Generate normally distributed errors with standard deviation 3.0
  std::ranlux24 gen;
  std::normal_distribution<FloatType> noise(0.0, 3.0);
  constexpr int n = 10;
  VectorX errors(n);
  errors.setZero();
  for (int i = 0; i < n; ++i)
  {
    errors(i) = noise(gen);
  }

  VectorX errors_scaled = HuberWeightFunction<FloatType>::weightVectorized(errors);

  //! @todo: add checks.
  VLOG(1) << errors.transpose();
  VLOG(1) << errors_scaled.transpose();
}

ZE_UNITTEST_ENTRYPOINT
