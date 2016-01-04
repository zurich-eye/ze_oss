#pragma once

#include <sys/stat.h>

#include <string>

namespace ze {

inline bool fileExists(const std::string& filename)
{
  struct stat buf;
  return stat(filename.c_str(), &buf) != -1;
}

inline bool isDir(const std::string& filename)
{
  struct stat buf;
  if(0 == stat(filename.c_str(), &buf)) {
    return S_ISDIR(buf.st_mode);
  }
  return false;
}

} // namespace ze
