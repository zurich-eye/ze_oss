#include <iostream>
#include <string>

#include <ze/common/test_entrypoint.h>
#include <ze/common/benchmark.h>

int foo(int x, int y)
{
  return x*x + y*y;
}

void printOut(const std::string& s)
{
  VLOG(10) << "Run dummy benchmark";
}

TEST(BenchmarkTest, testInterface)
{
  using namespace ze;
  auto fun1 = std::bind(printOut, "a");
  runTimingBenchmark(fun1, 5, 2);

  auto fun2 = std::bind(foo, 1, 2);
  int64_t duration_ns = runTimingBenchmark(fun2, 1000, 100, "foo", true);
}

ZE_UNITTEST_ENTRYPOINT
