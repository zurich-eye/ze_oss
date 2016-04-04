#pragma once

#include <chrono>
#include <ctime>   // std::localtime
#include <iomanip> // std::setw
#include <unordered_map>
#include <string>
#include <sstream>

#include <ze/common/time_conversions.h>
#include <ze/common/types.h>
#include <ze/common/running_statistics.h>

namespace ze {

//------------------------------------------------------------------------------
//! Simple timing utilty.
class Timer
{
public:
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using ns = std::chrono::nanoseconds;
  using ms = std::chrono::milliseconds;

  //! The constructor directly starts the timer.
  Timer()
    : start_time_(Clock::now())
  {}

  //! Starts the timer.
  inline void start()
  {
    start_time_ = Clock::now();
  }

  //! Stop timer and get nanoseconds passed.
  inline int64_t stopAndGetNanoseconds()
  {
    const TimePoint end_time(Clock::now());
    ns duration = std::chrono::duration_cast<ns>(end_time - start_time_);
    return duration.count();
  }

  //! Stops timer and returns duration in milliseconds.
  inline FloatType stop()
  {
    return nanosecToMillisecTrunc(stopAndGetNanoseconds());
  }

private:
  TimePoint start_time_;
};

//------------------------------------------------------------------------------
//! Collect statistics over multiple timings in milliseconds.
class TimedScope;

class TimerStatistics
{
public:
  inline void start()
  {
    t_.start();
  }

  //! Using the concept of "Initialization is Resource Acquisition" idiom, this
  //! function returns a timer object. When this timer object is destructed,
  //! the timer is stopped.
  inline TimedScope timeScope();

  inline FloatType stop()
  {
    FloatType t = t_.stop();
    stat_.addSample(t);
    return t;
  }

  inline FloatType numTimings() const { return stat_.numSamples(); }
  inline FloatType accumulated() const { return stat_.sum(); }
  inline FloatType min() const { return stat_.min(); }
  inline FloatType max() const { return stat_.max(); }
  inline FloatType mean() const { return stat_.mean(); }
  inline FloatType variance() const { return stat_.var(); }
  inline FloatType standarDeviation() const { return stat_.std(); }
  inline void reset() { stat_.reset(); }
  inline const RunningStatistics& statistics() const { return stat_; }

private:
  Timer t_;
  RunningStatistics stat_;
};

//! This object is return from TimerStatistics::timeScope()
class TimedScope
{
public:
  TimedScope() = delete;

  TimedScope(TimerStatistics* timer)
    : timer_(timer)
  {
    timer_->start();
  }

  ~TimedScope()
  {
    timer_->stop();
  }
private:
  TimerStatistics* timer_;
};

inline TimedScope TimerStatistics::timeScope()
{
  return TimedScope(this);
}

//------------------------------------------------------------------------------
//! Collect statistics over multiple timings in milliseconds.
//! Usage:
//! ze::TimerCollection timers;
//! timers["name"].start()
//! ...
//! timers["name"].stop()
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

  inline size_t numTimers() const { return timers_.size(); }

  void saveToFile(const std::string& directory, const std::string& filename);

  //!@{
  //! Timer iteration:
  typedef Timers::value_type value_type;
  typedef Timers::const_iterator const_iterator;
  Timers::const_iterator begin() const { return timers_.begin(); }
  Timers::const_iterator end() const { return timers_.end(); }
  //!@}

private:
  Timers timers_;
};

//! Print Timer Collection:
std::ostream& operator<<(std::ostream& out, const TimerCollection& timers);

//------------------------------------------------------------------------------
// Utilities:

//! Get nanoseconds since 1.1.1970
int64_t getCurrentNanosecondTime();

//! Get a formated string of the current time, hour, minute and second
std::string getCurrentTimeStr();

} // end namespace ze
