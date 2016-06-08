#include <ze/common/benchmark.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/random.hpp>
#include <ze/common/running_statistics.h>

TEST(RandomTests, testRandomSampling)
{
  using namespace ze;

  // Deterministic sampling results always the same series of random numbers.
  EXPECT_EQ(sampleFromUniformIntDistribution<uint8_t>(true), 140u);
  EXPECT_EQ(sampleFromUniformIntDistribution<uint8_t>(true), 151u);
  EXPECT_EQ(sampleFromUniformIntDistribution<uint8_t>(true), 183u);

  EXPECT_EQ(sampleFromUniformIntDistribution<int>(true), 209652396);
  EXPECT_EQ(sampleFromUniformIntDistribution<int>(true), 398764591);
  EXPECT_EQ(sampleFromUniformIntDistribution<int>(true), 924231285);

  EXPECT_NEAR(sampleFromUniformRealDistribution<double>(true), 0.592844, 1e-5);
  EXPECT_NEAR(sampleFromUniformRealDistribution<double>(true), 0.844265, 1e-5);
  EXPECT_NEAR(sampleFromUniformRealDistribution<double>(true), 0.857945, 1e-5);

  EXPECT_NEAR(sampleFromNormalDistribution<double>(true, 1.0, 4.0), 5.4911797, 1e-5);
  EXPECT_NEAR(sampleFromNormalDistribution<double>(true, 1.0, 4.0), 1.2834369, 1e-5);
  EXPECT_NEAR(sampleFromNormalDistribution<double>(true, 1.0, 4.0), -4.689303, 1e-5);

  EXPECT_TRUE (flipCoin(true, 0.7));
  EXPECT_FALSE(flipCoin(true, 0.7));
  EXPECT_FALSE(flipCoin(true, 0.7));
  EXPECT_FALSE(flipCoin(true, 0.7));
  EXPECT_TRUE (flipCoin(true, 0.7));
  EXPECT_TRUE (flipCoin(true, 0.7));

  // Non-deterministic sampling, always results in different numbers:
  EXPECT_NE(sampleFromUniformIntDistribution<int>(false), 209652396);
  EXPECT_NE(sampleFromUniformIntDistribution<int>(false), 398764591);
  EXPECT_NE(sampleFromUniformIntDistribution<int>(false), 924231285);

  // Test mean and standard deviation of normal distribution.
  {
    RunningStatistics statistics;
    for (int i = 0; i < 10000; ++i)
    {
      statistics.addSample(sampleFromNormalDistribution<double>(false, 2.0, 5.0));
    }
    EXPECT_NEAR(statistics.mean(), 2.0, 0.2);
    EXPECT_NEAR(statistics.std(),  5.0, 0.2);
  }

  // Test coin flips.
  {
    RunningStatistics statistics;
    for (int i = 0; i < 10000; ++i)
    {
      statistics.addSample(static_cast<int>(flipCoin(false, 0.2)));
    }
    EXPECT_NEAR(statistics.mean(), 0.2, 0.2);
  }
}

TEST(RandomTests, testDistribution)
{
  using namespace ze;

  // Deterministic sampling results always the same series of random numbers.
  {
    auto f = uniformDistribution<uint8_t>(true);
    EXPECT_EQ(f(), 140u);
    EXPECT_EQ(f(), 151u);
    EXPECT_EQ(f(), 183u);
  }

  // Deterministic sampling results always the same series of random numbers.
  {
    auto f = uniformDistribution<double>(true, 1.0, 2.0);
    EXPECT_NEAR(f(), 1.59284, 1e-5);
    EXPECT_NEAR(f(), 1.84427, 1e-5);
    EXPECT_NEAR(f(), 1.85795, 1e-5);
  }

  // Deterministic sampling results always the same series of random numbers.
  {
    auto f = normalDistribution<float>(true, 3.0, 5.0);
    EXPECT_NEAR(f(), 14.06103, 1e-5);
    EXPECT_NEAR(f(), 8.81539, 1e-5);
    EXPECT_NEAR(f(), 6.87001, 1e-5);
  }
}

TEST(RandomTests, benchmark)
{
  using namespace ze;

  auto lambda1 = [&]()
  {
    int sum = 0;
    for (int i = 0; i < 100000; ++i)
      sum += sampleFromUniformIntDistribution<uint8_t>(false);
  };
  runTimingBenchmark(lambda1, 10, 10, "sampleSeparately", true);

  auto lambda2 = [&]()
  {
    int sum = 0;
    auto dist = uniformDistribution<uint8_t>(false);
    for (int i = 0; i < 100000; ++i)
      sum += dist();
  };
  runTimingBenchmark(lambda2, 10, 10, "sampleFromDistribution", true);
}

ZE_UNITTEST_ENTRYPOINT
