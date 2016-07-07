#include <cmath>
#include <utility>
#include <ze/common/types.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/statistics.h>
#include <ze/common/random_matrix.hpp>

TEST(StatisticsTest, testMedian)
{
  Eigen::VectorXd x(5);
  x << 1, 2, 3, 4, 5;
  auto m = ze::median(x);
  EXPECT_DOUBLE_EQ(m.first, 3);
}

TEST(StatisticsTest, testMeasurementCovariance)
{
  using namespace ze;

  // Generate a random distribution matrix of known covariance.
  Vector3 variances;
  variances << 2.0, 3.0, 4.0;
  RandomVectorSampler<3>::Ptr sampler(
        RandomVectorSampler<3>::variances(variances));

  MatrixX measurements(3, 100000);
  for (int i = 0; i < 100000; ++i)
  {
    measurements.col(i) = sampler->sample();
  }

  Matrix3 cov = measurementCovariance(measurements);
  Matrix3 ref = Vector3(2.0, 3.0, 4.0).asDiagonal();

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(cov, ref, 1e-1));
}

ZE_UNITTEST_ENTRYPOINT
