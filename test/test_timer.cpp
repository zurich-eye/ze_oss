#include <chrono>
#include <thread>
#include <iostream>

#include <ze/common/test_entrypoint.h>
#include <ze/common/timer.h>
#include <ze/common/timer_collection.h>
#include <ze/common/timer_statistics.h>

TEST(TimerTests, testTimerInterface)
{
  ze::Timer t;
  std::this_thread::sleep_for(ze::Timer::ms(10));
  EXPECT_NEAR(t.stop(), 10.0, 0.5);
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
  EXPECT_NEAR(timer.mean(), 10.0, 0.1);
  EXPECT_NEAR(timer.accumulated(), 100.0, 1.0);
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
  EXPECT_NEAR(timer.mean(), 10.0, 0.1);
  EXPECT_NEAR(timer.accumulated(), 100.0, 1.0);
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


ZE_UNITTEST_ENTRYPOINT
