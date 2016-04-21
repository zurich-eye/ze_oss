#pragma once

#include <unordered_map>
#include <string>
#include <sstream>

#include <ze/common/types.h>
#include <ze/common/running_statistics.h>

namespace ze {

/*! Collect statistics over multiple timings in milliseconds.
 *
 * Usage:
\code{.cpp}
  ze::StatisticsCollection stats;
  stats["name"].addSample(12);
  stats["name"].addSample(10);
  FloatType variance = stats["name"].var();
\endcode
*/
class StatisticsCollection
{
public:
  using Collection = std::unordered_map<std::string, RunningStatistics>;

  StatisticsCollection() = default;
  ~StatisticsCollection() = default;

  inline RunningStatistics& operator[](const std::string& name)
  {
    return collection_[name];
  }

  inline RunningStatistics& operator[](std::string&& name)
  {
    return collection_[std::forward<std::string>(name)];
  }

  inline size_t size() const { return collection_.size(); }

  //! Saves statistics to file in YAML format.
  void saveToFile(const std::string& directory, const std::string& filename);

  //! @name Statistics iterator.
  //! @{
  typedef Collection::value_type value_type;
  typedef Collection::const_iterator const_iterator;
  Collection::const_iterator begin() const { return collection_.begin(); }
  Collection::const_iterator end() const { return collection_.end(); }
  //! @}

private:
  Collection collection_;
};

//! Print statistics collection.
std::ostream& operator<<(std::ostream& out, const StatisticsCollection& collection);

} // end namespace ze
