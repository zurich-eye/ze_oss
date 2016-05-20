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

namespace ze {
namespace test {

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
  auto random_val = ze::test::getRandomGenerator<float>();
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

} // test namespace
} // ze namespace

TEST(IMPCuCoreTestSuite,sumTest_32fC1)
{
  const size_t width = 752;
  const size_t height = 480;
  ze::ImageRaw32fC1 im =
      ze::test::generateRandomImage(width, height);
  double gt_sum = ze::test::gtSum(im);

  IMP_CUDA_CHECK();
  ze::cu::ImageGpu32fC1 cu_im(im);
  IMP_CUDA_CHECK();
  double cu_sum = ze::cu::sum(cu_im);
  EXPECT_NEAR(gt_sum, cu_sum, 0.1);
  printf("GT sum: %f\n", gt_sum);
  printf("GPU sum: %f\n", cu_sum);
}

TEST(IMPCuCoreTestSuite,sumByReductionTestConstImg_32fC1)
{
  const size_t width = 752;
  const size_t height = 480;
  const float val = 0.1f;
  ze::ImageRaw32fC1 im =
      ze::test::generateConstantImage(width, height, val);
  printf("test image has been filled with constant value %f\n", val);
  double gt_sum = static_cast<double>(width*height) * val;

  IMP_CUDA_CHECK();
  ze::cu::ImageGpu32fC1 cu_im(im);
  IMP_CUDA_CHECK();
  dim3 num_threads_per_block;
  dim3 num_blocks_per_grid;
  num_threads_per_block.x = 16;
  num_threads_per_block.y = 16;
  num_blocks_per_grid.x = 4;
  num_blocks_per_grid.y = 4;
  ze::cu::ImageReducer<float> reducer(
        num_threads_per_block,
        num_blocks_per_grid);
  double cu_sum = reducer.sum(cu_im.cuData(), cu_im.stride(), cu_im.width(), cu_im.height());
  EXPECT_NEAR(gt_sum, cu_sum, 0.01);
  printf("GT sum: %f\n", gt_sum);
  printf("GPU sum: %f\n", cu_sum);
}

TEST(IMPCuCoreTestSuite,sumByReductionTestRndImg_32fC1)
{
  const size_t width = 752;
  const size_t height = 480;
  const float val = 0.1f;
  ze::ImageRaw32fC1 im =
      ze::test::generateRandomImage(width, height);
  double gt_sum = ze::test::gtSum(im);

  IMP_CUDA_CHECK();
  ze::cu::ImageGpu32fC1 cu_im(im);
  IMP_CUDA_CHECK();
  dim3 num_threads_per_block;
  dim3 num_blocks_per_grid;
  num_threads_per_block.x = 16;
  num_threads_per_block.y = 16;
  num_blocks_per_grid.x = 4;
  num_blocks_per_grid.y = 4;
  ze::cu::ImageReducer<float> reducer(
        num_threads_per_block,
        num_blocks_per_grid);
  double cu_sum = reducer.sum(cu_im.cuData(), cu_im.stride(), cu_im.width(), cu_im.height());
  EXPECT_NEAR(gt_sum, cu_sum, 0.01);
  printf("GT sum: %f\n", gt_sum);
  printf("GPU sum: %f\n", cu_sum);
}
