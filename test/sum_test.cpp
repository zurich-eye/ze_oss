#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <cfloat>
#include <iostream>
#include <random>
#include <functional>
#include <limits>
#include <imp/core/image_raw.hpp>
#include <imp/cu_core/cu_math.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/common/test_utils.h>

TEST(IMPCuCoreTestSuite,sumTest_32fC1)
{
  // setup random number generator
  auto random_val = ze::getRandomGenerator<float>();

  const size_t width = 752;
  const size_t height = 480;
  ze::ImageRaw32fC1 im(width,height);

  double gt_sum = 0.0;
  for (size_t y=0; y<height; ++y)
  {
    for (size_t x=0; x<width; ++x)
    {
      float random_value = random_val();
      im[y][x] = random_value;
      gt_sum += im.pixel(x, y);
    }
  }
  ze::cu::ImageGpu32fC1 cu_im(im);
  double cu_sum = ze::cu::sum(cu_im);
  EXPECT_NEAR(gt_sum, cu_sum, 0.1);
  printf("GT sum: %f\n", gt_sum);
  printf("GPU sum: %f\n", cu_sum);
}
