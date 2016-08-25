// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/common/test_utils.hpp>

#include <cstdlib>
#include <ze/common/logging.hpp>
#include <ze/common/types.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/common/string_utils.hpp>
#include <ze/common/file_utils.hpp>

namespace ze {

std::string getTestDataDir(const std::string& dataset_name)
{
  const char* datapath_dir = std::getenv("ZE_TEST_DATA_PATH");
  CHECK(datapath_dir != nullptr)
      << "Did you download the ze_test_data repository and set "
      << "the ZE_TEST_DATA_PATH environment variable?";

  std::string path(datapath_dir);
  CHECK(isDir(path)) << "Folder does not exist: " << path;
  path = path + "/data/" + dataset_name;
  CHECK(isDir(path)) << "Dataset does not exist: " << path;
  return path;
}

std::map<int64_t, Transformation> loadIndexedPosesFromCsv(const std::string& filename)
{
  std::map<int64_t, Transformation> poses;
  std::ifstream fs;
  openFileStream(filename, &fs);
  std::string line;
  while(std::getline(fs, line))
  {
    std::vector<std::string> items = splitString(line, ',');
    CHECK_EQ(items.size(), 8u);
    int64_t stamp = std::stoll(items[0]);
    Vector3 t(std::stod(items[1]), std::stod(items[2]), std::stod(items[3]));
    Quaternion q(std::stod(items[7]), std::stod(items[4]), std::stod(items[5]), std::stod(items[6]));
    poses[stamp] = Transformation(q, t);
  }
  return poses;
}

void loadDepthmapFromFile(
    const std::string& filename, const size_t data_size, float* data)
{
  CHECK_NOTNULL(data);
  std::ifstream fs;
  openFileStream(filename, &fs);
  float* data_ptr = data;
  for(size_t i = 0; i < data_size; ++i, ++data_ptr)
  {
    fs >> (*data_ptr);
    CHECK(fs.peek() != '\n' || i == data_size - 1)
        << "Did not read full depthmap. Num elements read = " <<  i
        << " of expected " << data_size;
  }
}

} // namespace ze
