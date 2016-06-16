#include <ze/common/test_entrypoint.h>

#include <ze/imu/imu_buffer.h>

TEST(ImuBufferTest, testBuffer)
{
  using namespace ze;
  std::shared_ptr<ImuIntrinsicModelCalibrated> intrinsics =
      std::make_shared<ImuIntrinsicModelCalibrated>();
  std::shared_ptr<ImuNoiseNone> noise = std::make_shared<ImuNoiseNone>();

  AccelerometerModel::Ptr a_model =
      std::make_shared<AccelerometerModel>(intrinsics, noise);
  GyroscopeModel::Ptr g_model =
      std::make_shared<GyroscopeModel>(intrinsics, noise);

  ImuModel::Ptr model(std::make_shared<ImuModel>(a_model, g_model));

  ImuBufferLinear5000 buffer(model);
  Vector3 a1 = Vector3::Random();
  Vector3 a2 = Vector3::Random();
  Vector3 a3 = Vector3::Random();
  buffer.insertAccelerometerMeasurement(10, a1);
  buffer.insertAccelerometerMeasurement(20, a2);
  buffer.insertAccelerometerMeasurement(30, a3);

  Vector3 w1 = Vector3::Random();
  Vector3 w2 = Vector3::Random();
  Vector3 w3 = Vector3::Random();
  buffer.insertGyroscopeMeasurement(11, w1);
  buffer.insertGyroscopeMeasurement(21, w2);
  buffer.insertGyroscopeMeasurement(33, w3);

  Vector6 i1 = Vector6::Random();
  buffer.insertImuMeasurement(40, i1);

  Vector6 out1;
  EXPECT_TRUE(buffer.get(15, out1));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR((a1 + a2)/2, out1.head<3>(3), 1e-6));
  EXPECT_TRUE(buffer.get(16, out1));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR((w1 + w2)/2, out1.tail<3>(3), 1e-6));

  // Between interpolation:
  ImuAccGyr values;
  ImuStamps stamps;
  std::tie(stamps, values) = buffer.getBetweenValuesInterpolated(15, 35);

  ImuStamps ref_stamps(4); ref_stamps << 15, 21, 33, 35;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(ref_stamps, stamps, 1e-8));

  // Check interpolation of accelerometer
  buffer.get(stamps(0), out1);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(values.col(0), out1, 1e-6));
  buffer.get(stamps(1), out1);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(values.col(1), out1, 1e-6));
  buffer.get(stamps(2), out1);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(values.col(2), out1, 1e-6));
  buffer.get(stamps(3), out1);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(values.col(3), out1, 1e-6));

  // Test Interpolated bounds:
  // lower out of bound:
  std::tie(stamps, values) = buffer.getBetweenValuesInterpolated(0, 35);
  EXPECT_EQ(0, stamps.size());
  EXPECT_EQ(0, values.cols());

  std::tie(stamps,values) = buffer.getBetweenValuesInterpolated(15, 50);
  EXPECT_EQ(0, stamps.size());
  EXPECT_EQ(0, values.cols());

  std::tie(stamps, values) = buffer.getBetweenValuesInterpolated(0, 50);
  EXPECT_EQ(0, stamps.size());
  EXPECT_EQ(0, values.cols());

  // Test Bounds Get:
  EXPECT_FALSE(buffer.get(0, out1));
  EXPECT_FALSE(buffer.get(50, out1));
}

ZE_UNITTEST_ENTRYPOINT
