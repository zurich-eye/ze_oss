#include <string>
#include <iostream>

#include <ze/common/test/entrypoint.h>
#include <ze/common/test/utils.h>
#include <ze/data_provider/data_provider_csv.h>
#include <ze/data_provider/data_provider_rosbag.h>

TEST(DataProviderTests, testCsv)
{
  std::string data_dir = ze::common::getTestDataDir("csv_dataset");
  EXPECT_FALSE(data_dir.empty());

  ze::DataProviderCsv dp(data_dir+"/data", "imu0", { {"cam0", 0} });

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

  dp.spin();

  EXPECT_EQ(expect_imu_stamp, 0);
  EXPECT_EQ(expect_cam_stamp, 0);
  EXPECT_EQ(num_cam_measurements, 5);
  EXPECT_EQ(num_imu_measurements, 69);
}

TEST(DataProviderTests, testRosbag)
{
  std::string data_dir = ze::common::getTestDataDir("rosbag_euroc_snippet");
  std::string bag_filename = data_dir + "/dataset.bag";
  EXPECT_TRUE(ze::common::fileExists(bag_filename));

  ze::DataProviderRosbag dp(bag_filename, "/imu0", { {"/cam0/image_raw", 0},
                                                     {"/cam1/image_raw", 1} });

  size_t num_imu_measurements = 0;
  dp.registerImuCallback(
        [&](int64_t stamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr)
  {
    ++num_imu_measurements;
  });

  size_t num_cam0_measurements = 0, num_cam1_measurements = 0;
  dp.registerCameraCallback(
        [&](int64_t stamp, const cv::Mat& img, size_t cam_idx)
  {
    if(cam_idx == 0)
      ++num_cam0_measurements;
    if(cam_idx == 1)
      ++num_cam1_measurements;
  });

  dp.spin();

  EXPECT_EQ(num_cam0_measurements, 21);
  EXPECT_EQ(num_cam1_measurements, 20);
  EXPECT_EQ(num_imu_measurements, 210);
}



ZE_UNITTEST_ENTRYPOINT
