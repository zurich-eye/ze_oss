// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/common/test_entrypoint.hpp>

#include <ze/imu/imu_model.hpp>

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

TEST(ImuModelTest, testUndistortion)
{
  using namespace ze;
  Matrix3 M;
  M << 0., 0., 1.,
       0., 1., 0.,
       1., 0., 0.;
  std::shared_ptr<ImuIntrinsicModelScaleMisalignment> intrinsics =
      std::make_shared<ImuIntrinsicModelScaleMisalignment>(
        0.0,
        ImuIntrinsicModel::UndefinedRange,
        Vector3::Zero(),
        M);
  std::shared_ptr<ImuNoiseNone> noise = std::make_shared<ImuNoiseNone>();

  AccelerometerModel::Ptr a_model =
      std::make_shared<AccelerometerModel>(intrinsics, noise);
  GyroscopeModel::Ptr g_model =
      std::make_shared<GyroscopeModel>(intrinsics, noise);

  ImuModel model(a_model, g_model);

  Vector3 gyro_measurement;
  Vector3 accel_measurement;
  accel_measurement << 1., 2., 3.;
  gyro_measurement << 4., 5., 6.;
  Vector6 undistorted_measurement = model.undistort(accel_measurement, gyro_measurement);
  EXPECT_EQ(undistorted_measurement, (Vector6() << 3., 2., 1., 6., 5., 4.).finished());

  EXPECT_EQ(a_model, model.accelerometerModel());
  EXPECT_EQ(g_model, model.gyroscopeModel());
}
ZE_UNITTEST_ENTRYPOINT
