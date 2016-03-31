#include <iostream>
#include <vector>
#include <chrono>

#include <ze/common/thread_pool.h>

int main(int argc, char** argv)
{
  ze::ThreadPool pool(4);
  std::vector< std::future<int> > results;

  for (int i = 0; i < 8; ++i)
  {
    results.emplace_back(
      pool.enqueue([i] {
        std::cout << "started job " << i << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "stopped job " << i << std::endl;
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
