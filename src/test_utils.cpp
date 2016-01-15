#include <ze/common/test_utils.h>

#include <cstdlib>
#include <glog/logging.h>
#include <ze/common/types.h>
#include <ze/common/path_utils.h>
#include <ze/common/string_utils.h>
#include <ze/common/file_utils.h>

namespace ze {

std::string getTestDataDir(const std::string& dataset_name)
{
  const char* datapath_dir = std::getenv("ZE_TEST_DATA_PATH");
  CHECK_NOTNULL(datapath_dir);
  std::string path(datapath_dir);
  CHECK(isDir(path)) << "Folder does not exist: " << path << "."
                     << "Did you download the ze_test_data repository and set "
                     << "the ZE_TEST_DATA_PATH environment variable?";
  path = path + "/data/" + dataset_name;
  CHECK(isDir(path)) << "Dataset does not exist: " << path;
  return path;
}

std::map<size_t, Transformation> loadIndexedPosesFromCsv(const std::string& filename)
{
  std::map<size_t, Transformation> poses;
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

} // namespace ze
