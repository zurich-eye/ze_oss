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
#include <imp/cu_core/cu_image_reduction.cuh>
#include <ze/common/benchmark.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_utils.h>

namespace ze {

double gtSum(const ze::ImageRaw32fC1& im)
{
  double sum{0};
  for (size_t y = 0; y < im.height(); ++y)
  {
    for (size_t x = 0; x < im.width(); ++x)
    {
      sum += im.pixel(x, y);
    }
  }
  return sum;
}

ze::ImageRaw32fC1 generateRandomImage(size_t width, size_t height)
{
  auto random_val = ze::getRandomGenerator<float>();
  ze::ImageRaw32fC1 im(width,height);
  for (size_t y = 0; y < im.height(); ++y)
  {
    for (size_t x = 0; x < im.width(); ++x)
    {
      float random_value = random_val();
      im[y][x] = random_value;
    }
  }
  return im;
}

ze::ImageRaw32fC1 generateConstantImage(size_t width, size_t height, float val)
{
  ze::ImageRaw32fC1 im(width,height);
  for (size_t y = 0; y < im.height(); ++y)
  {
    for (size_t x = 0; x < im.width(); ++x)
    {
      im[y][x] = val;
    }
  }
  return im;
}

} // ze namespace

TEST(IMPCuCoreTestSuite, sumByReductionTestConstImg_32fC1)
{
  const size_t width = 752;
  const size_t height = 480;
  const float val = 0.1f;
  ze::ImageRaw32fC1 im =
      ze::generateConstantImage(width, height, val);
  VLOG(1) << "test image has been filled with the constant value " << val;
  double gt_sum = static_cast<double>(width*height) * val;

  IMP_CUDA_CHECK();
  ze::cu::ImageGpu32fC1 cu_im(im);
  IMP_CUDA_CHECK();

  ze::cu::ImageReducer<ze::Pixel32fC1> reducer;
  double cu_sum;
  auto sumReductionLambda = [&](){
    cu_sum = reducer.sum(cu_im);
  };
  reducer.sum(cu_im); //! Warm-up
  ze::runTimingBenchmark(
        sumReductionLambda,
        20, 40,
        "sum using parallel reduction",
        true);
  const double tolerance = 0.01;
  EXPECT_NEAR(gt_sum, cu_sum, tolerance);
  VLOG(1) << "GT sum: " << std::fixed << gt_sum;
  VLOG(1) << "GPU sum: " << std::fixed << cu_sum;
  VLOG(1) << "Test tolerance: " << std::fixed << tolerance;
}


TEST(IMPCuCoreTestSuite, sumByReductionTestRndImg_32fC1)
{
  const size_t width = 752;
  const size_t height = 480;
  ze::ImageRaw32fC1 im =
      ze::generateRandomImage(width, height);
  double gt_sum = ze::gtSum(im);

  IMP_CUDA_CHECK();
  ze::cu::ImageGpu32fC1 cu_im(im);
  IMP_CUDA_CHECK();

  ze::cu::ImageReducer<ze::Pixel32fC1> reducer;
  double cu_sum;
  auto sumReductionLambda = [&](){
    cu_sum = reducer.sum(cu_im);
  };
  reducer.sum(cu_im); //! Warm-up
  ze::runTimingBenchmark(
        sumReductionLambda,
        20, 40,
        "sum using parallel reduction",
        true);
  const double tolerance = 0.01;
  EXPECT_NEAR(gt_sum, cu_sum, tolerance);
  VLOG(1) << "GT sum: " << std::fixed << gt_sum;
  VLOG(1) << "GPU sum: " << std::fixed << cu_sum;
  VLOG(1) << "Test tolerance: " << std::fixed << tolerance;
}

TEST(IMPCuCoreTestSuite, countEqualByReductionTestConstImg_32sC1)
{
  const size_t width = 752;
  const size_t height = 480;
  ze::ImageRaw32sC1 im(width, height);
  const size_t gt{4};
  const int probe_val{7};
  //! Fill GT pixels with gt probe values
  im.pixel(width/2, height/2) = probe_val;
  im.pixel(width/4, height/4) = probe_val;
  im.pixel(width/2, height/4) = probe_val;
  im.pixel(0, 0) = probe_val;

  IMP_CUDA_CHECK();
  ze::cu::ImageGpu32sC1 cu_im(im);
  IMP_CUDA_CHECK();

  ze::cu::ImageReducer<ze::Pixel32sC1> reducer;
  size_t cu_res;
  auto countEqualReductionLambda = [&](){
    cu_res = reducer.countEqual(cu_im, probe_val);
  };
  reducer.countEqual(cu_im, probe_val); //! Warm-up
  ze::runTimingBenchmark(
        countEqualReductionLambda,
        20, 40,
        "countEqual using parallel reduction",
        true);
  EXPECT_EQ(gt, cu_res);
  VLOG(1) << "GT countEqual: " << gt;
  VLOG(1) << "GPU countEqual: " << cu_res;
}
