#pragma once

#include <sstream>
#include <string>
#include <algorithm>
#include <vector>

namespace ze {
namespace common {

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

inline std::vector<std::string> splitString(const std::string& s, char delim)
{
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> items;
  while (std::getline(ss, item, delim)) {
    items.push_back(item);
  }
  return items;
}

} // namespace common
} // namespace ze
