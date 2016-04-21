#include <iostream>
#include <vector>
#include <chrono>

#include <ze/common/logging.hpp>
#include <ze/common/thread_pool.h>

int main(int argc, char** argv)
{
  ze::ThreadPool pool(4);
  std::vector< std::future<int> > results;

  for (int i = 0; i < 8; ++i)
  {
    results.emplace_back(
      pool.enqueue([i] {
        VLOG(1) << "started job " << i;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        VLOG(1) << "stopped job " << i;
        return i*i;
      })
    );
  }

  for (auto&& result : results)
  {
    std::cout << result.get() << ' ';
  }
  std::cout << std::endl;

  return 1;
}
