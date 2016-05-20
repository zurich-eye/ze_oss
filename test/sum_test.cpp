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
#include <ze/common/test_utils.h>
#include <ze/common/file_utils.h>
#include <ze/common/benchmark.h>

namespace ze {
namespace test_sum {

constexpr uint32_t g_num_iter_per_epoch = 20;
constexpr uint32_t g_num_epochs = 40;

template<class T>
typename std::enable_if<std::is_integral<T>::value, std::function<T()> >::type
getRandomGenerator()
{
  std::default_random_engine generator;
  std::uniform_int_distribution<T> distribution(0, 255);
  auto random_val = std::bind(distribution, generator);
  return random_val;
}

template<class T>
typename std::enable_if<!std::is_integral<T>::value, std::function<T()> >::type
getRandomGenerator()
{
  std::default_random_engine generator;
  std::uniform_real_distribution<T> distribution(0, 1);
  auto random_val = std::bind(distribution, generator);
  return random_val;
}

double gtSum(const ze::ImageRaw32fC1& im)
{
  double sum{0};
  for (size_t y=0; y<im.height(); ++y)
  {
    for (size_t x=0; x<im.width(); ++x)
    {
      sum += im.pixel(x, y);
    }
  }
  return sum;
}

ze::ImageRaw32fC1 generateRandomImage(size_t width, size_t height)
{
  auto random_val = getRandomGenerator<float>();
  ze::ImageRaw32fC1 im(width,height);
  for (size_t y=0; y<im.height(); ++y)
  {
    for (size_t x=0; x<im.width(); ++x)
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
  for (size_t y=0; y<im.height(); ++y)
  {
    for (size_t x=0; x<im.width(); ++x)
    {
      im[y][x] = val;
    }
  }
  return im;
}

} // test_sum namespace
} // ze namespace

TEST(IMPCuCoreTestSuite,sumTest_32fC1)
{
  const size_t width = 752;
  const size_t height = 480;
  ze::ImageRaw32fC1 im =
      ze::test_sum::generateRandomImage(width, height);
  double gt_sum = ze::test_sum::gtSum(im);

  IMP_CUDA_CHECK();
  ze::cu::ImageGpu32fC1 cu_im(im);
  IMP_CUDA_CHECK();
  double cu_sum;
  auto sumTextureLambda = [&](){
      cu_sum = ze::cu::sum(cu_im);
  };
  ze::cu::sum(cu_im); //! Warm-up
  ze::runTimingBenchmark(
          sumTextureLambda,
          ze::test_sum::g_num_iter_per_epoch,
          ze::test_sum::g_num_epochs,
          "sum using texture memory",
          true);
  const double tolerance = 0.1;
  EXPECT_NEAR(gt_sum, cu_sum, tolerance);
  printf("GT sum: %f\n", gt_sum);
  printf("GPU sum: %f\n", cu_sum);
  printf("Test tolerance: %f\n", tolerance);
}
