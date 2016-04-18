#pragma once

#include <chrono>
#include <functional>
#include <limits>

#include <ze/common/logging.hpp>
#include <ze/common/types.h>
#include <ze/common/time.h>

namespace ze {

template <typename Lambda>
uint64_t runTimingBenchmark(
    const Lambda& benchmark_fun, uint32_t num_iter_per_epoch, uint32_t num_epochs,
    const std::string& benchmark_name = "", bool print_results = false)
{
  // Define lambda that runs experiment num_iter_per_epoch times and measures.
  auto executeFunMultipleTimes = [=]() -> uint64_t
  {
    // Measurement starts.
    auto const r1 = std::chrono::high_resolution_clock::now();
    for (uint32_t i = 0; i < num_iter_per_epoch; ++i)
    {
      benchmark_fun();
    }
    auto const r2 = std::chrono::high_resolution_clock::now();
    // Measurement ends.

    return std::chrono::duration_cast<std::chrono::nanoseconds>(r2 - r1).count();
  };

  uint64_t min_time = std::numeric_limits<uint64_t>::max();
  for (uint32_t i = 0; i < num_epochs; ++i)
  {
    // Call function.
    uint64_t timing = executeFunMultipleTimes();

    // According to Andrei Alexandrescu, the best measure is to take the minimum.
    // See talk: https://www.youtube.com/watch?v=vrfYLlR8X8k
    min_time = std::min(timing, min_time);
  }

  if(print_results)
  {
    VLOG(1) << "Benchmark: " << benchmark_name << "\n"
            << "> Time for " << num_iter_per_epoch << " iterations: "
            << nanosecToMillisec(min_time) << " milliseconds\n"
            << "> Time for 1 iteration: "
            << nanosecToMillisec(min_time) / num_iter_per_epoch << " milliseconds";
  }

  return min_time;
}


} // namespace ze
