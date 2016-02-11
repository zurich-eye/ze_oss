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
  ze::PoseSeries gt_poses;
  gt_poses.load(ze::joinPath(test_data_dir, "traj_gt.csv"));

  // Load estimated trajectory.
  ze::SWEResultSeries es_poses;
  es_poses.load(ze::joinPath(test_data_dir, "traj_es.csv"));

  // Load matches to compare against
  std::map<int64_t, int64_t> matches;
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

  // Now loop through all estimated poses and find closest groundtruth-stamp.
  auto & gt_buffer = gt_poses.getBuffer();
  auto & es_buffer = es_poses.getBuffer();
  es_buffer.lock();
  int n_skipped = 0, n_checked = 0;
  for(const auto & pose : es_buffer.data())
  {
    ze::Vector7 matched_gt_pose;
    int64_t matched_gt_stamp;
    if(!ze::findNearestTimeStamp(gt_buffer,
                                 pose.first,
                                 matched_gt_stamp,
                                 matched_gt_pose,
                                 max_difference_sec,
                                 offset_sec))
    {
      ++n_skipped;
    }
    else
    {
      EXPECT_LE(std::abs(pose.first-matched_gt_stamp), ze::secToNanosec(max_difference_sec));
      EXPECT_EQ(matches.find(pose.first)->second, matched_gt_stamp);
      ++n_checked;
    }
  }
  es_buffer.unlock();
  EXPECT_EQ(es_buffer.size(), n_checked+n_skipped);
}

ZE_UNITTEST_ENTRYPOINT
