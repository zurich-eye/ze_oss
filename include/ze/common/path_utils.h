#pragma once

#include <sys/stat.h>
#include <string>
#include <fstream>

#include <ze/common/string_utils.h>

namespace ze {

inline bool fileExists(const std::string& filename)
{
  std::ifstream infile(filename);
  return infile.good();
}

inline bool isDir(const std::string& filename)
{
  struct stat buf;
  if(0 == stat(filename.c_str(), &buf))
  {
    return S_ISDIR(buf.st_mode);
  }
  return false;
}

inline std::string joinPath(const std::string& s1, const std::string& s2)
{
  return std::string(ensureRightSlash(s1) + ensureNoLeftSlash(s2));
}

inline std::string joinPath(const std::string& s1, const std::string& s2,
                            const std::string& s3)
{
  return joinPath(joinPath(s1, s2), s3);
}

inline std::string joinPath(const std::string& s1, const std::string& s2,
                            const std::string& s3, const std::string& s4)
{
  return joinPath(joinPath(s1, s2), joinPath(s3, s4));
}

inline std::string getFileName(const std::string& path)
{
  return path.substr(path.find_last_of('/') + 1);
}

} // namespace ze
