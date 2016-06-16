#include <string>
#include <iostream>

#include <imp/core/image_base.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/data_provider/data_provider_csv.hpp>
#include <ze/data_provider/data_provider_rosbag.hpp>
#include <ze/data_provider/camera_imu_synchronizer_unsync.hpp>
#include <ze/imu/imu_model.h>

namespace ze {
// a dummy data provider
class DataProviderDummy : public DataProviderBase
{
public:
  DataProviderDummy(): DataProviderBase(DataProviderType::Rosbag) {}
  void spin() {}
  virtual bool spinOnce() { return true; }
  virtual bool ok() const { return true; }
  virtual size_t imuCount() const
  {
    return imu_count_;
  };
  virtual size_t cameraCount() const
  {
    return camera_count_;
  }
  size_t imu_count_;
  size_t camera_count_;
};
}

TEST(CameraImuSynchronizerUnsyncTest, testFunctionality)
{
  using namespace ze;
  // fake data provider with two cameras
  DataProviderDummy data_provider;
  data_provider.camera_count_ = 1;
  data_provider.imu_count_ = 1;

  // get an imu model
  std::shared_ptr<ImuIntrinsicModelCalibrated> intrinsics =
      std::make_shared<ImuIntrinsicModelCalibrated>();
  std::shared_ptr<ImuNoiseNone> noise = std::make_shared<ImuNoiseNone>();

  AccelerometerModel::Ptr a_model =
      std::make_shared<AccelerometerModel>(intrinsics, noise);
  GyroscopeModel::Ptr g_model =
      std::make_shared<GyroscopeModel>(intrinsics, noise);

  ImuModel model(a_model, g_model);

  CameraImuSynchronizerUnsync sync(data_provider,
                                  { std::make_shared<ImuModel>(a_model, g_model) });

  size_t measurements = 0u;
  sync.registerCameraImuCallback(
        [&](const StampedImages& images,
            const ImuStampsVector& imu_timestamps,
            const ImuAccGyrVector& imu_measurements)
        {
          ++measurements;
          EXPECT_EQ(1, images.size());
          EXPECT_EQ(1, imu_timestamps.size());
          EXPECT_EQ(1, imu_measurements.size());
        }
  );

  int64_t stamp1 = 1403636579763555580;
  int64_t stamp2 = 1403636579763555590;
  int64_t stamp3 = 1403636579763555600;

  auto img1 = std::make_shared<ImageRaw8uC1>(1, 1);

  sync.addGyroData(stamp1, Vector3(), 0);
  sync.addAccelData(stamp1 + 1, Vector3(), 0);
  EXPECT_EQ(0, measurements);

  sync.addImgData(stamp2, img1, 0);
  EXPECT_EQ(0, measurements);

  sync.addGyroData(stamp3, Vector3(), 0);
  EXPECT_EQ(0, measurements);
  sync.addAccelData(stamp3 + 1, Vector3(), 0);
  EXPECT_EQ(1, measurements);
}

ZE_UNITTEST_ENTRYPOINT
