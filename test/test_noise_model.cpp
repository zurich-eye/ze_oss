// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/common/test_entrypoint.hpp>

#include <ze/imu/imu_noise_model.hpp>

TEST(NoiseModelTests, testWhiteBrownian)
{
  using namespace ze;
  ImuNoiseWhiteBrownian::Ptr noise = std::make_shared<ImuNoiseWhiteBrownian>(
                                    0.1, 2u, 0.2);

  ASSERT_DOUBLE_EQ(0.1, noise->noiseDensity());
  ASSERT_EQ(2, noise->bandwidth());
  ASSERT_DOUBLE_EQ(0.2, noise->biasNoiseDensity());
  ASSERT_TRUE(ImuNoiseType::WhiteBrownian == noise->type());
}

ZE_UNITTEST_ENTRYPOINT
