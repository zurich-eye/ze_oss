#include <iostream>
#include <gtest/gtest.h>

#include <ze/common/types.h>
#include <ze/common/buffer.h>
#include <ze/common/file_utils.h>
#include <ze/common/path_utils.h>
#include <ze/common/time.h>
#include <ze/common/test_utils.h>
#include <ze/ts_match.h>
#include <ze/common/csv_trajectory.h>
#include <ze/common/test_entrypoint.h>

// Matching parameters
constexpr double offset_sec = 0.0;
constexpr double max_difference_sec = 0.02;

TEST(TimeStampMatcher, matchTest)
{
  std::string test_data_dir = ze::getTestDataDir("ze_ts_matching");

  // Load groundtruth.
  ze::Buffer<ze::FloatType,3> gt_buffer;
  {
    std::ifstream fs;
    ze::openFileStream(ze::joinPath(test_data_dir, "traj_gt.csv"), &fs);
    std::string line;
    while(std::getline(fs, line))
    {
      if('%' != line.at(0) && '#' != line.at(0))
      {
        std::vector<std::string> items = ze::splitString(line, ',');
        EXPECT_GE(items.size(), 4u);
        int64_t stamp = std::stoll(items[0]);
        ze::Vector3 pos(std::stod(items[1]), std::stod(items[2]), std::stod(items[3]));
        gt_buffer.insert(stamp, pos);
      }
    }
  }

  // Load estimate.
  std::vector<std::pair<int64_t, ze::Transformation>> es_poses;
  {
    std::ifstream fs;
    ze::openFileStream(ze::joinPath(test_data_dir, "traj_es.csv"), &fs);
    std::string line;
    int64_t offset_nsec = ze::secToNanosec(offset_sec);
    while(std::getline(fs, line))
    {
      if('%' != line.at(0) && '#' != line.at(0))
      {
        std::vector<std::string> items = ze::splitString(line, ',');
        EXPECT_GE(items.size(), 4u);
        int64_t stamp = std::stoll(items[0]);
        ze::Vector3 pos(std::stod(items[1]), std::stod(items[2]), std::stod(items[3]));
        ze::Quaternion rot(std::stod(items[7]), std::stod(items[4]), std::stod(items[5]), std::stod(items[6]));
        es_poses.push_back(std::make_pair(stamp + offset_nsec, ze::Transformation(rot, pos)));
      }
    }
  }

  // Load matches to compare against
  std::map<int64_t, int64_t> matches;
  {
    std::ifstream fs;
    ze::openFileStream(ze::joinPath(test_data_dir, "python_matches.csv"), &fs);
    std::string line;
    while(std::getline(fs, line))
    {
      if('%' != line.at(0) && '#' != line.at(0))
      {
        std::vector<std::string> items = ze::splitString(line, ',');
        EXPECT_GE(items.size(), 2u);
        int64_t stamp_es = std::stoll(items[0]);
        int64_t stamp_gt = std::stoll(items[1]);
        matches[stamp_es] = stamp_gt;
      }
    }
  }

  // Now loop through all estimated poses and find closest groundtruth-stamp.
  int n_skipped = 0, n_checked = 0;
  for(const auto & it : es_poses)
  {
    ze::Vector3 gt_pos;
    int64_t gt_stamp;
    if(!ze::findNearestTimeStamp(gt_buffer,
                                 it.first,
                                 gt_stamp,
                                 gt_pos,
                                 max_difference_sec,
                                 offset_sec))
    {
      ++n_skipped;
    }
    else
    {
      EXPECT_LE(std::abs(it.first-gt_stamp), ze::secToNanosec(max_difference_sec));
      EXPECT_EQ(matches.find(it.first)->second, gt_stamp);
      ++n_checked;
    }
  }
  EXPECT_EQ(es_poses.size(), n_checked+n_skipped);
}

ZE_UNITTEST_ENTRYPOINT
