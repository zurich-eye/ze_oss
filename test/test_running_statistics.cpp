#include <ze/common/test_entrypoint.h>
#include <ze/common/running_statistics.h>

TEST(RunningStatisticsTest, test)
{
  ze::RunningStatistics stat;
  stat.addSample(1.1);
  stat.addSample(2.2);
  stat.addSample(3.3);
  stat.addSample(2.7);
  stat.addSample(4.5);
  EXPECT_EQ(stat.numSamples(), 5u);
  EXPECT_DOUBLE_EQ(stat.max(), 4.5);
  EXPECT_DOUBLE_EQ(stat.min(), 1.1);
  EXPECT_DOUBLE_EQ(stat.sum(), 13.8);
  EXPECT_DOUBLE_EQ(stat.mean(), 2.76);
  EXPECT_DOUBLE_EQ(stat.var(), 1.5979999999999994);
  EXPECT_DOUBLE_EQ(stat.std(), 1.2641202474448383);
  VLOG(1) << "Statistics:\n" << stat;
}

ZE_UNITTEST_ENTRYPOINT
