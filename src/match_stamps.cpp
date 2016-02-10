#include <iostream>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <ze/common/types.h>
#include <ze/common/buffer.h>
#include <ze/common/file_utils.h>
#include <ze/common/path_utils.h>
#include <ze/common/transformation.h>
#include <ze/common/time.h>

DEFINE_string(data_dir, "", "Path to data");
DEFINE_string(filename_estimate, "", "Filename of estimated trajectory.");
DEFINE_string(filename_groundtruth, "", "Filename of groundtruth trajectory.");

DEFINE_double(offset_sec, 0.0, "time offset added to the timestamps of the estimate");
DEFINE_double(max_difference_sec, 0.02, "maximally allowed time difference for matching entries");

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Load groundtruth.
  ze::Buffer<ze::FloatType,3> gt_buffer;
  {
    std::ifstream fs;
    ze::openFileStream(ze::joinPath(FLAGS_data_dir, FLAGS_filename_groundtruth), &fs);
    std::string line;
    while(std::getline(fs, line))
    {
      if('%' != line.at(0) && '#' != line.at(0))
      {
        std::vector<std::string> items = ze::splitString(line, ',');
        //std::cout << "GT LINE: " << line << std::endl;
        //std::cout << "\t TS: " << items[0] << std::endl;
        CHECK_GE(items.size(), 4u);
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
    ze::openFileStream(ze::joinPath(FLAGS_data_dir, FLAGS_filename_estimate), &fs);
    std::string line;
    int64_t offset_nsec = ze::secToNanosec(FLAGS_offset_sec);
    while(std::getline(fs, line))
    {
      if('%' != line.at(0) && '#' != line.at(0))
      {
        std::vector<std::string> items = ze::splitString(line, ',');
        CHECK_GE(items.size(), 4u);
        int64_t stamp = std::stoll(items[0]);
        ze::Vector3 pos(std::stod(items[1]), std::stod(items[2]), std::stod(items[3]));
        ze::Quaternion rot(std::stod(items[7]), std::stod(items[4]), std::stod(items[5]), std::stod(items[6]));
        es_poses.push_back(std::make_pair(stamp + offset_nsec, ze::Transformation(rot, pos)));
      }
    }
  }

  // Now loop through all estimated poses and find closest groundtruth-stamp.
  {
    int64_t max_diff_nsec = ze::secToNanosec(FLAGS_max_difference_sec);
    int n_skipped = 0;
    std::ofstream fs;
    ze::openOutputFileStream(ze::joinPath(FLAGS_data_dir, "matched_poses.csv"), &fs);
    fs << "# gt-stamp, gt-x, gt-y, gt-z, es-stamp, es-x, es-y, es-z, es-qx, es-qy, es-qz, es-qw\n";
    std::string separator = ", ";
    for(const std::pair<int64_t, ze::Transformation>& it : es_poses)
    {
      bool success;
      ze::Vector3 gt_pos;
      int64_t gt_stamp;
      std::tie(gt_stamp, gt_pos, success) = gt_buffer.getNearestValue(it.first);
      CHECK(success);
      if(std::abs(gt_stamp - it.first) > max_diff_nsec)
      {
        ++n_skipped;
        continue;
      }

      // Write to file
      fs << gt_stamp << separator
         << gt_pos.x() << separator
         << gt_pos.y() << separator
         << gt_pos.z() << separator
         << it.first << separator
         << it.second.getPosition().x() << separator
         << it.second.getPosition().y() << separator
         << it.second.getPosition().z() << "\n";
    }

    VLOG(1) << "Wrote " << es_poses.size() - n_skipped << " matched poses to file"
            << ". Skipped " << n_skipped << " because of too large time difference.";
  }

  return 0;
}
