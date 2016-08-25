// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/vi_simulation/imu_bias_simulator.hpp>
#include <ze/common/test_entrypoint.hpp>

TEST(ImuBiasTest, testConstantImuBias)
{
  using namespace ze;

  Vector3 acc_bias(1, 2, 3);
  Vector3 gyr_bias(4, 5, 6);
  ConstantBiasSimulator bias(acc_bias, gyr_bias);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(acc_bias, bias.accelerometer(0), 1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(gyr_bias, bias.gyroscope(0), 1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(acc_bias, bias.accelerometer(10), 1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(gyr_bias, bias.gyroscope(10), 1e-8));
}

TEST(ImuBiasTest, testContinuousBias)
{
  using namespace ze;
  ContinuousBiasSimulator bias(Vector3(1e-4, 1e-4, 1e-4),
                      Vector3(1e-3, 1e-3, 1e-3),
                      10, 20, 200);
  bias.accelerometer(10);
  bias.gyroscope(20);
}

ZE_UNITTEST_ENTRYPOINT
