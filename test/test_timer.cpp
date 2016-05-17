#include <chrono>
#include <thread>
#include <iostream>

#include <ze/common/string_utils.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/timer.h>
#include <ze/common/timer_collection.h>
#include <ze/common/timer_statistics.h>

TEST(TimerTests, testTimerInterface)
{
  ze::Timer t;
  std::this_thread::sleep_for(ze::Timer::ms(10));
  EXPECT_NEAR(t.stopAndGetMilliseconds(), 10.0, 0.5);
}

TEST(TimerTests, testTimerStatistics)
{
  ze::TimerStatistics timer;
  for(int i = 0; i < 10; ++i)
  {
    timer.start();
    std::this_thread::sleep_for(ze::Timer::ms(10));
    timer.stop();
  }
  EXPECT_EQ(timer.numTimings(), 10u);
  EXPECT_NEAR(timer.mean(), 10.0, 0.5);
  EXPECT_NEAR(timer.accumulated(), 100.0, 10.0);
  EXPECT_GT(timer.max(), timer.min());
}

TEST(TimerTests, testTimerScope)
{
  ze::TimerStatistics timer;
  for(int i = 0; i < 10; ++i)
  {
    auto t = timer.timeScope();
    std::this_thread::sleep_for(ze::Timer::ms(10));
  }
  EXPECT_EQ(timer.numTimings(), 10u);
  EXPECT_NEAR(timer.mean(), 10.0, 0.5);
  EXPECT_NEAR(timer.accumulated(), 100.0, 10.0);
  EXPECT_GT(timer.max(), timer.min());
}

TEST(TimerTests, testTimerCollection)
{
  ze::TimerCollection timers;
  for(int i = 0; i < 10; ++i)
  {
    timers["foo"].start();
    std::this_thread::sleep_for(ze::Timer::ms(50));
    timers["foo"].stop();

    timers["bar"].start();
    std::this_thread::sleep_for(ze::Timer::ms(10));
    timers["bar"].stop();
  }
  VLOG(1) << timers;
  EXPECT_NO_FATAL_FAILURE(timers.saveToFile("/tmp", "test_timer.yaml"));
  EXPECT_EQ(timers.size(), 2u);
}

namespace ze {

enum MyTimers
{
  Foo,
  Bar,
  dimension
};

template<typename TimerEnum>
class NewTimerCollection
{
public:

  NewTimerCollection(const std::string& timer_names_comma_separated)
    : names_(splitString(timer_names_comma_separated, ','))
  {
    CHECK_EQ(names_.size(), timers_.size());
  }

  NewTimerCollection(const std::vector<std::string>& timer_names)
    : names_(timer_names)
  {
    CHECK_EQ(names_.size(), timers_.size());
  }

  inline TimerStatistics& operator[](TimerEnum t)
  {
    return timers_[static_cast<uint32_t>(t)];
  }

  inline const TimerStatistics& operator[](TimerEnum t) const
  {
    return timers_[static_cast<uint32_t>(t)];
  }

  inline size_t size() const { return timers_.size(); }

  std::array<TimerStatistics, static_cast<uint32_t>(TimerEnum::dimension)> timers_;
  std::vector<std::string> names_;

};

// Problem: string for enum type.

} // namespace ze

#define DECLARE_TIMER(classname, membername, ...)                           \
  enum class classname : uint32_t { __VA_ARGS__, dimension };               \
  ze::NewTimerCollection<classname> membername { #__VA_ARGS__ }

TEST(TimerTests, testNewTimerCollection)
{
  using namespace ze;

  DECLARE_TIMER(FrontendTimer, timer_, Visualization, Tracking, Mapping);

  EXPECT_EQ(timer_.size(), 3u);

  timer_[FrontendTimer::Visualization].start();
  std::this_thread::sleep_for(ze::Timer::ms(50));
  timer_[FrontendTimer::Visualization].stop();
}

ZE_UNITTEST_ENTRYPOINT
