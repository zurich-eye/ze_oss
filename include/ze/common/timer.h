#pragma once

#include <chrono>

#include <ze/common/time_conversions.h>
#include <ze/common/types.h>

namespace ze {

///! Simple timing utilty.
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

} // end namespace ze
