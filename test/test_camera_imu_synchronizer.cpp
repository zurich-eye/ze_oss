#include <string>
#include <iostream>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/data_provider/data_provider_csv.h>
#include <ze/data_provider/data_provider_rosbag.h>
#include <ze/data_provider/camera_imu_synchronizer.h>
#include <imp/core/image_base.hpp>
#include <imp/bridge/opencv/image_cv.hpp>

TEST(CameraImuSynchronizerTest, testCsv)
{
  using namespace ze;

  std::string data_dir = getTestDataDir("csv_dataset");
  EXPECT_FALSE(data_dir.empty());

  DataProviderCsv dp(data_dir+"/data", "imu0", { {"cam0", 0} });
  CameraImuSynchronizer sync(1, 1.0);
  sync.subscribeDataProvider(dp);
  sync.registerCameraImuCallback(
        [&](const StampedImages& images,
            const ImuStamps& imu_timestamps,
            const ImuAccGyr& imu_measurements)
  {
    static int64_t last_img_stamp = 1403636579758555392;
    VLOG(1) << "Image stamp = " << images[0].first << ", "
            << "IMU-min = " << imu_timestamps(0) << ", "
            << "IMU-max = " << imu_timestamps(imu_timestamps.size()-1) << "\n";
    EXPECT_EQ(last_img_stamp,  imu_timestamps(0));
    EXPECT_EQ(images[0].first, imu_timestamps(imu_timestamps.size()-1));
    EXPECT_EQ(imu_timestamps.size(), imu_measurements.cols());
    last_img_stamp = images[0].first;
  });

  dp.spin();
}

ZE_UNITTEST_ENTRYPOINT
