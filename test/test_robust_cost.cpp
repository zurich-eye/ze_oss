#include <cmath>
#include <random>
#include <utility>
#include <ze/common/test/entrypoint.h>
#include <ze/geometry/robust_cost.h>

TEST(RobustCostTest, testScaleEstimators)
{
  // Generate normally distributed errors with standard deviation 3.0
  std::ranlux24 gen;
  std::normal_distribution<double> noise(0.0, 3.0);
  constexpr int n = 1000;
  Eigen::VectorXd errors(n);
  errors.setZero();
  for(int i = 0; i < n; ++i)
    errors(i) = noise(gen);

  double s1 = ze::UnitScaleEstimator<double>::compute(errors);
  EXPECT_DOUBLE_EQ(s1, 1.0);

  double s2 = ze::NormalDistributionScaleEstimator<double>::compute(errors);
  EXPECT_TRUE(std::abs(s2 - 3.0) < 0.2);

  double s3 = ze::MADScaleEstimator<double>::compute(errors);
  EXPECT_TRUE(std::abs(s3 - 3.0) < 0.2);
}

TEST(RobustCostTest, testWeightFunctions)
{
  // Generate normally distributed errors with standard deviation 3.0
  std::ranlux24 gen;
  std::normal_distribution<double> noise(0.0, 3.0);
  constexpr int n = 10;
  Eigen::VectorXd errors(n);
  errors.setZero();
  for(int i = 0; i < n; ++i)
    errors(i) = noise(gen);

  Eigen::VectorXd errors_scaled =
      ze::HuberWeightFunction<double>::weightVectorized(errors);

  std::cout << errors.transpose() << std::endl;
  std::cout << errors_scaled.transpose() << std::endl;
}

ZE_UNITTEST_ENTRYPOINT
