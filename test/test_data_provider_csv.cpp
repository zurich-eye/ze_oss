#include <string>
#include <iostream>

#include <ze/common/test/entrypoint.h>
#include <ze/common/test/utils.h>
#include <ze/data_provider/data_provider_csv.h>

TEST(DataProviderTests, testCsv)
{
  std::string data_dir = ze::common::getTestDataDir("csv_dataset");
  EXPECT_FALSE(data_dir.empty());

  ze::DataProviderCsv dp(data_dir+"/data", {0}, {0}, {});

  size_t num_imu_measurements = 0;
  int64_t expect_imu_stamp = 1403636579758555392;
  dp.registerImuCallback(
        [&](int64_t stamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr)
  {
    ++num_imu_measurements;
    if(stamp == expect_imu_stamp)
      expect_imu_stamp = 0;
  });

  size_t num_cam_measurements = 0;
  int64_t expect_cam_stamp = 1403636579763555584;
  dp.registerCameraCallback(
        [&](int64_t stamp, const cv::Mat& img, size_t cam_idx)
  {
    ++num_cam_measurements;
    if(stamp == expect_cam_stamp)
      expect_cam_stamp = 0;
  });

  while(!dp.finished())
    dp.spinOnceBlocking();

  EXPECT_EQ(expect_imu_stamp, 0);
  EXPECT_EQ(expect_cam_stamp, 0);
  EXPECT_EQ(num_cam_measurements, 5);
  EXPECT_EQ(num_imu_measurements, 69);
}

ZE_UNITTEST_ENTRYPOINT
