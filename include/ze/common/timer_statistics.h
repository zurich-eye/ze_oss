#pragma once

#include <ze/common/running_statistics.h>
#include <ze/common/timer.h>
#include <ze/common/types.h>

namespace ze {

// fwd
class TimedScope;

//! Collect statistics over multiple timings in milliseconds.
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
  // Returning a pointer to this should be safe, as inserting more elements in
  // the unordered map does not invalidate any references to other elements in
  // the unordered map (http://stackoverflow.com/questions/16781886).
  return TimedScope(this);
}

} // end namespace ze
