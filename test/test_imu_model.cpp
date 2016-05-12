#include <ze/common/test_entrypoint.h>

#include <ze/imu/imu_model.h>

TEST(ImuModelTest, testImu)
{
  using namespace ze;
  std::shared_ptr<ImuIntrinsicModelCalibrated> intrinsics =
      std::make_shared<ImuIntrinsicModelCalibrated>();
  std::shared_ptr<ImuNoiseNone> noise = std::make_shared<ImuNoiseNone>();

  AccelerometerModel::Ptr a_model =
      std::make_shared<AccelerometerModel>(intrinsics, noise);
  GyroscopeModel::Ptr g_model =
      std::make_shared<GyroscopeModel>(intrinsics, noise);

  ImuModel model(a_model, g_model);

  EXPECT_EQ(a_model, model.accelerometerModel());
  EXPECT_EQ(g_model, model.gyroscopeModel());
}

ZE_UNITTEST_ENTRYPOINT
