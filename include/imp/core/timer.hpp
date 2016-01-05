#ifndef IMP_TIMER_HPP
#define IMP_TIMER_HPP

#include <iostream>
#include <chrono>
#include <string>
#include <glog/logging.h>

namespace imp {

class SingleShotTimer
{
public:
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = Clock::time_point ;
  using Milliseconds = std::chrono::milliseconds;
  using Nanoseconds = std::chrono::nanoseconds;
  using Seconds = std::chrono::seconds;

public:
  SingleShotTimer(const std::string& name)
    : name_(name)
    , start_(Clock::now())
  { ; }

  SingleShotTimer()
    : SingleShotTimer("SingleShotTimer")
  { ; }


  virtual ~SingleShotTimer()
  {
    std::cout << "[" << name_ << "] "
              << std::fixed << this->elapsedNs().count() << " ns" << std::endl;
  }

  /**
   * @brief reset resets the timer
   */
  void reset() noexcept
  {
    start_ = Clock::now();
  }

  /**
   * @brief elapsedMs returns the elapsed \a Milliseconds
   */
  Milliseconds elapsedMs() const noexcept
  {
    return std::chrono::duration_cast<Milliseconds>(Clock::now() - start_);
  }

  /**
   * @brief elapsedNs returns the elapsed \a Nanoseconds
   */
  Nanoseconds elapsedNs() const noexcept
  {
    return std::chrono::duration_cast<Nanoseconds>(Clock::now() - start_);
  }

  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits>& operator<< (
      std::basic_ostream<T, Traits>& out, const SingleShotTimer& timer)
  {
    return out << timer.elapsedNs().count() << " ns";
  }

private:
  std::string name_;
  TimePoint start_;
};


} // namespace imp

#endif // IMP_TIMER_HPP

