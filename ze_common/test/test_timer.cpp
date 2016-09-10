// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <chrono>
#include <thread>
#include <iostream>

#include <ze/common/string_utils.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/timer.hpp>
#include <ze/common/timer_collection.hpp>
#include <ze/common/timer_statistics.hpp>

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
  DECLARE_TIMER(TestTimer, timers, foo, bar);

  for(int i = 0; i < 10; ++i)
  {
    timers[TestTimer::foo].start();
    std::this_thread::sleep_for(ze::Timer::ms(50));
    timers[TestTimer::foo].stop();

    timers[TestTimer::bar].start();
    std::this_thread::sleep_for(ze::Timer::ms(10));
    timers[TestTimer::bar].stop();
  }
  VLOG(1) << timers;
  EXPECT_NO_FATAL_FAILURE(timers.saveToFile("/tmp", "test_timer.yaml"));
  EXPECT_EQ(timers.size(), 2u);
}

ZE_UNITTEST_ENTRYPOINT
