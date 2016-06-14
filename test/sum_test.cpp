#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <cfloat>
#include <iostream>
#include <functional>
#include <limits>
#include <imp/core/image_raw.hpp>
#include <imp/cu_core/cu_math.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/common/benchmark.h>
#include <ze/common/random.hpp>
#include <ze/common/test_utils.h>

TEST(IMPCuCoreTestSuite, sumTest_32fC1)
{
  // setup random number generator
  auto random_val = ze::uniformDistribution<float>(ZE_DETERMINISTIC);

  const size_t width = 752;
  const size_t height = 480;
  ze::ImageRaw32fC1 im(width,height);

  double gt_sum = 0.0;
  for (size_t y = 0; y < height; ++y)
  {
    for (size_t x = 0; x < width; ++x)
    {
      float random_value = random_val();
      im[y][x] = random_value;
      gt_sum += im.pixel(x, y);
    }
  }
  double cu_sum;
  ze::cu::ImageGpu32fC1 cu_im(im);
  auto sumTextureLambda = [&](){
    cu_sum = ze::cu::sum(cu_im);
  };
  ze::cu::sum(cu_im); //! Warm-up
  ze::runTimingBenchmark(
        sumTextureLambda,
        20, 40,
        "sum using texture memory",
        true);
  const double tolerance = 0.15;

  EXPECT_NEAR(gt_sum, cu_sum, tolerance);
  VLOG(1) << "GT sum: " << std::fixed << gt_sum;
  VLOG(1) << "GPU sum: " << std::fixed << cu_sum;
  VLOG(1) << "Test tolerance: " << std::fixed << tolerance;
}
