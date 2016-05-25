#include <ze/common/test_entrypoint.h>
#include <ze/common/running_statistics.h>
#include <ze/common/running_statistics_collection.h>

TEST(RunningStatisticsTest, testRunningStatistics)
{
  ze::RunningStatistics stat;
  stat.addSample(1.1);
  stat.addSample(2.2);
  stat.addSample(3.3);
  stat.addSample(2.7);
  stat.addSample(4.5);
  EXPECT_EQ(stat.numSamples(), 5u);
  EXPECT_FLOATTYPE_EQ(stat.max(), 4.5);
  EXPECT_FLOATTYPE_EQ(stat.min(), 1.1);
  EXPECT_FLOATTYPE_EQ(stat.sum(), 13.8);
  EXPECT_FLOATTYPE_EQ(stat.mean(), 2.76);
  EXPECT_FLOATTYPE_EQ(stat.var(), 1.5979999999999994);
  EXPECT_FLOATTYPE_EQ(stat.std(), 1.2641202474448383);
  VLOG(1) << "Statistics:\n" << stat;
}

TEST(RunningStatisticsTest, testCollection)
{
  using namespace ze;

  DECLARE_STATISTICS(Statistics, stats, foo, bar);
  stats[Statistics::foo].addSample(1.0);
  stats[Statistics::foo].addSample(1.0);
  stats[Statistics::foo].addSample(1.0);
  stats[Statistics::bar].addSample(2.0);
  EXPECT_EQ(stats[Statistics::foo].numSamples(), 3u);
  EXPECT_EQ(stats[Statistics::bar].numSamples(), 1u);
  EXPECT_FLOATTYPE_EQ(stats[Statistics::foo].mean(), 1.0);
  EXPECT_FLOATTYPE_EQ(stats[Statistics::bar].mean(), 2.0);
  VLOG(1) << stats;
}

ZE_UNITTEST_ENTRYPOINT

