#include <cmath>
#include <utility>
#include <ze/common/test/entrypoint.h>
#include <ze/common/statistics.h>

TEST(StatisticsTest, testMedian)
{
  Eigen::VectorXd x(5);
  x << 1, 2, 3, 4, 5;
  auto m = ze::median(x);
  EXPECT_DOUBLE_EQ(m.first, 3);
}

ZE_UNITTEST_ENTRYPOINT
