#include <ze/common/test_entrypoint.h>

#include <ze/imu/gyroscope_model.h>
#include <ze/imu/imu_intrinsic_model.h>
#include <ze/imu/imu_noise_model.h>

TEST(GyroscopeModelTest, tesGyroscope)
{
  using namespace ze;
  std::shared_ptr<ImuIntrinsicModelCalibrated> intrinsics =
      std::make_shared<ImuIntrinsicModelCalibrated>();
  std::shared_ptr<ImuNoiseNone> noise = std::make_shared<ImuNoiseNone>();

  GyroscopeModel model(intrinsics, noise);

  EXPECT_EQ(intrinsics, model.intrinsicModel());
  EXPECT_EQ(noise, model.noiseModel());
}

ZE_UNITTEST_ENTRYPOINT
