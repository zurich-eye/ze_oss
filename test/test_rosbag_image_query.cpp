// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <string>
#include <iostream>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/ros/rosbag_image_query.hpp>
#include <imp/core/image_base.hpp>

TEST(RosbagImageQueryTests, testImageQuery)
{
  using namespace ze;

  std::string data_dir = getTestDataDir("rosbag_euroc_snippet");
  std::string bag_filename = joinPath(data_dir, "dataset.bag");
  RosbagImageQuery rosbag(bag_filename);

  {
    // stamp of the first image in the bag:
    int64_t nsec = nanosecFromSecAndNanosec(1403636617, 863555500);
    auto res = rosbag.getStampedImageAtTime("/cam0/image_raw", nsec);
    EXPECT_EQ(res.first, nsec);
    EXPECT_TRUE(res.second != nullptr);
  }

  {
    // stamp of some image in the bag:
    int64_t nsec = nanosecFromSecAndNanosec(1403636618, 463555500);
    auto res = rosbag.getStampedImageAtTime("/cam0/image_raw", nsec);
    EXPECT_EQ(res.first, nsec);
    EXPECT_TRUE(res.second != nullptr);
  }
}

ZE_UNITTEST_ENTRYPOINT
