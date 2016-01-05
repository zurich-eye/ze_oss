#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>

#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>


TEST(IMPCuCoreTestSuite,imageGpuSimpleTest)
{
  //
  // 2D image
  //
  int width = 123;
  int height = 324;
  imp::cu::ImageGpu8uC1 im(width, height);
  IMP_CUDA_CHECK();
  ASSERT_EQ(width, im.width());
  ASSERT_EQ(height, im.height());
  IMP_CUDA_CHECK();
}
