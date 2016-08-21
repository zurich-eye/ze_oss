#include <iostream>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <ze/common/types.h>
#include <ze/common/buffer.h>
#include <ze/common/file_utils.h>
#include <ze/common/transformation.h>
#include <ze/common/time_conversions.h>

DEFINE_string(data_dir, ".", "Path to data");
DEFINE_string(filename_es, "traj_es.csv", "Filename of estimated trajectory.");
DEFINE_string(filename_gt, "traj_gt.csv", "Filename of groundtruth trajectory.");
DEFINE_string(filename_matches, "traj_es_gt_matches.csv", "Filename of matched timestamps");
DEFINE_uint64(stamp_index_estimate, 0, "Index of timestamp in estimate file.");
DEFINE_uint64(stamp_index_groundtruth, 0, "Index of timestamp in estimate file.");
DEFINE_double(offset_sec, 0.0, "time offset added to the timestamps of the estimate");
DEFINE_double(max_difference_sec, 0.02, "maximally allowed time difference for matching entries");

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  // Load groundtruth.
  ze::Buffer<ze::real_t,1> gt_stamps;
  {
    ze::Vector1 dummy; // We misuse the Buffer class to easily find the closest timestamp.
    std::ifstream fs;
    ze::openFileStream(ze::joinPath(FLAGS_data_dir, FLAGS_filename_gt), &fs);
    std::string line;
    while(std::getline(fs, line))
    {
      if('%' != line.at(0) && '#' != line.at(0) && 't' != line.at(0))
      {
        std::vector<std::string> items = ze::splitString(line, ',');
        CHECK_GE(items.size(), FLAGS_stamp_index_groundtruth + 1);
        int64_t stamp = std::stoll(items[FLAGS_stamp_index_groundtruth]);
        gt_stamps.insert(stamp, dummy);
      }
    }
  }

  // Load estimate stamps.
  std::vector<int64_t> es_stamps;
  {
    std::ifstream fs;
    ze::openFileStream(ze::joinPath(FLAGS_data_dir, FLAGS_filename_es), &fs);
    std::string line;
    int64_t offset_nsec = ze::secToNanosec(FLAGS_offset_sec);
    while(std::getline(fs, line))
    {
      if('%' != line.at(0) && '#' != line.at(0) && 't' != line.at(0))
      {
        std::vector<std::string> items = ze::splitString(line, ',');
        CHECK_GE(items.size(), FLAGS_stamp_index_estimate + 1);
        int64_t stamp = std::stoll(items[FLAGS_stamp_index_estimate]);
        es_stamps.push_back(stamp + offset_nsec);
      }
    }
  }

  // Now loop through all estimate stamps and find closest groundtruth-stamp.
  {
    int64_t max_diff_nsec = ze::secToNanosec(FLAGS_max_difference_sec);
    int n_skipped = 0;
    std::ofstream fs;
    ze::openOutputFileStream(ze::joinPath(FLAGS_data_dir, FLAGS_filename_matches), &fs);
    fs << "# es-stamp [nsec], gt-stamp [nsec]\n";
    for(const int64_t& es_stamp : es_stamps)
    {
      bool success;
      int64_t gt_stamp;
      std::tie(gt_stamp, std::ignore, success) = gt_stamps.getNearestValue(es_stamp);
      CHECK(success);
      if(std::abs(gt_stamp - es_stamp) > max_diff_nsec)
      {
        ++n_skipped;
        continue;
      }
      fs << es_stamp << ", " << gt_stamp << "\n";
    }

    VLOG(1) << "Wrote " << es_stamps.size() - n_skipped << " matched poses to file"
            << ". Skipped " << n_skipped << " because of too large time difference.";
  }

  return 0;
}
