// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
