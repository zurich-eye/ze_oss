#pragma once

#include <cstdlib>
#include <string>
#include <glog/logging.h>
#include <ze/common/path_utils.h>

namespace ze {
namespace common {

std::string getTestDataDir()
{
  const char* datapath_dir = std::getenv("ZE_TEST_DATA_PATH");
  CHECK_NOTNULL(datapath_dir) << "Environment variable ZE_TEST_DATA_PATH not set.";
  std::string path(datapath_dir);
  CHECK(common::isDir(path)) << "ZE_TEST_DATA_PATH is not a directory.";
  return path;
}

} // namespace common
} // namespace ze
