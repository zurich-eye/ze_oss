#pragma once

#include <unordered_map>
#include <string>
#include <sstream>

#include <ze/common/time_conversions.h>
#include <ze/common/timer_statistics.h>
#include <ze/common/types.h>
#include <ze/common/running_statistics.h>

namespace ze {

/*! Collect statistics over multiple timings in milliseconds.
 *
 * Usage with explicit start and stop:
\code{.cpp}
  ze::TimerCollection timers;
  timers["name"].start();
  ...
  timers["name"].stop();
\endcode
 * Or RAII-style, use a TimedScope object that stops the timer when it
 * goes out of scope.
\code{.cpp}
  {
    auto t = timers["name"].timeScope();
    ...
  }
\endcode
*/
class TimerCollection
{
public:
  using Timers = std::unordered_map<std::string, TimerStatistics>;

  TimerCollection() = default;
  ~TimerCollection() = default;

  inline TimerStatistics& operator[](const std::string& name)
  {
    return timers_[name];
  }

  inline TimerStatistics& operator[](std::string&& name)
  {
    return timers_[std::forward<std::string>(name)];
  }

  inline size_t size() const { return timers_.size(); }

  //! Saves timings to file in YAML format.
  void saveToFile(const std::string& directory, const std::string& filename);

  //! @name Timer iterator.
  //! @{
  typedef Timers::value_type value_type;
  typedef Timers::const_iterator const_iterator;
  Timers::const_iterator begin() const { return timers_.begin(); }
  Timers::const_iterator end() const { return timers_.end(); }
  //! @}

private:
  Timers timers_;
};

//! Print Timer Collection:
std::ostream& operator<<(std::ostream& out, const TimerCollection& timers);

} // end namespace ze
