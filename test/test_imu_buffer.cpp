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

  ImuAccGyr i1 = ImuAccGyr::Random();
  buffer.insertImuMeasurement(40, i1);

  ImuAccGyr out1;
  EXPECT_TRUE(buffer.get(15, out1));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR((a1 + a2)/2, out1.head<3>(3), 1e-6));
  EXPECT_TRUE(buffer.get(16, out1));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR((w1 + w2)/2, out1.tail<3>(3), 1e-6));

  // Between interpolation:
  ImuAccGyrContainer values;
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

TEST(ImuBufferTest, testOldestAndNewestTimestamp)
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

  int64_t oldest;
  int64_t newest;
  bool success;
  std::tie(oldest, newest, success) = buffer.getOldestAndNewestStamp();
  EXPECT_FALSE(success);

  buffer.insertImuMeasurement(1, ImuAccGyr::Random());

  std::tie(oldest, newest, success) = buffer.getOldestAndNewestStamp();
  EXPECT_TRUE(success);
  EXPECT_EQ(1, oldest);
  EXPECT_EQ(1, newest);

  buffer.insertImuMeasurement(2, ImuAccGyr::Random());

  std::tie(oldest, newest, success) = buffer.getOldestAndNewestStamp();
  EXPECT_TRUE(success);
  EXPECT_EQ(1, oldest);
  EXPECT_EQ(2, newest);

}

TEST(ImuBufferTest, testDifferentiation)
{

  using namespace ze;
  typedef Ringbuffer<real_t, 3, 100> RingBuffer_t;
  VectorX m;
  Vector6 ref;

  RingBuffer_t buffer;
  buffer.insert(10., Vector3::Zero());
  buffer.insert(20., 5. * Vector3::Ones());
  buffer.insert(30., 15. * Vector3::Ones());

  std::lock_guard<std::mutex> lock(buffer.mutex());

  //test differentiation between support points.
  m = InterpolatorDifferentiatorLinear::interpolate<RingBuffer_t>(&buffer, 15.);  ref << 2.5 * Vector3::Ones(), .5 * Vector3::Ones();
  ref << 2.5 * Vector3::Ones(), .5 * Vector3::Ones();
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(ref, m, 1e-10));

  //test differentiation at support point.
  //the expected derivative is the one of the segment starting with this point.
  m = InterpolatorDifferentiatorLinear::interpolate<RingBuffer_t>(&buffer, 20.);
  ref << 5. * Vector3::Ones(), 1. * Vector3::Ones();
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(ref, m, 1e-10));
}

ZE_UNITTEST_ENTRYPOINT
