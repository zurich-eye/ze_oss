#include <iostream>
#include <vector>
#include <chrono>

#include <ze/common/logging.hpp>
#include <ze/common/test_entrypoint.h>
#include <ze/common/thread_pool.hpp>

TEST(ThreadPoolTests, testThreadPool)
{
  ze::ThreadPool pool(4);
  std::vector<std::future<size_t>> results;

  for (size_t i = 0; i < 8; ++i)
  {
    results.emplace_back(
      pool.enqueue([i] {
        VLOG(1) << "started job " << i;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        VLOG(1) << "stopped job " << i;
        return i * i;
      })
    );
  }

  for (size_t i = 0u; i < results.size(); ++i)
  {
    CHECK_EQ(results[i].get(), i * i);
  }
}

ZE_UNITTEST_ENTRYPOINT
