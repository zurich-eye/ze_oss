#pragma once

#include <cstdlib>
#include <string>
#include <glog/logging.h>
#include <ze/common/path_utils.h>

namespace ze {

std::string getTestDataDir(const std::string& dataset_name)
{
  const char* datapath_dir = std::getenv("ZE_TEST_DATA_PATH");
  CHECK_NOTNULL(datapath_dir);
  std::string path(datapath_dir);
  CHECK(common::isDir(path)) << "Folder does not exist: " << path;
  path = path + "/data/" + dataset_name;
  CHECK(common::isDir(path)) << "Dataset does not exist: " << path;
  return path;
}

} // namespace ze
