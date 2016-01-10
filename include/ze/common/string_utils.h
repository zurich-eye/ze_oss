#pragma once

#include <sstream>
#include <string>
#include <algorithm>
#include <vector>

#include <glog/logging.h>

namespace ze {

inline std::string& leftTrimString(std::string& s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}

inline std::string& rightTrimString(std::string& s)
{
  s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
  return s;
}

inline std::string& trimString(std::string& s)
{
  return leftTrimString(rightTrimString(s));
}

inline std::string& ensureLeftSlash(std::string& s)
{
  CHECK_GE(s.size(), 1u);
  if(s[0] != '/')
  {
    s.insert(0, "/");
  }
  return s;
}

inline std::string ensureLeftSlash(const std::string& s)
{
  CHECK_GE(s.size(), 1u);
  std::string s_copy = s;
  if(s_copy[0] != '/')
  {
    s_copy.insert(0, "/");
  }
  return s_copy;
}

inline std::vector<std::string> splitString(const std::string& s, char delim)
{
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> items;
  while (std::getline(ss, item, delim))
  {
    items.push_back(item);
  }
  return items;
}

} // namespace ze
