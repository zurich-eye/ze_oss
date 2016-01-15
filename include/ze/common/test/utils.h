#pragma once

#include <map>
#include <cstdlib>
#include <string>
#include <glog/logging.h>
#include <ze/common/types.h>
#include <ze/common/path_utils.h>
#include <ze/common/string_utils.h>
#include <ze/common/transformation.h>

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
/*
std::map<size_t, Transformation> loadIndexedPosesFromCsv(const std::string& filename)
{

}
*/
} // namespace ze
