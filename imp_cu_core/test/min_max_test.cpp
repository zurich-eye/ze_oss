// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <ze/common/random.hpp>
#include <ze/common/test_utils.hpp>


TEST(IMPCuCoreTestSuite,minMaxTest_8uC1)
{
  auto random_val = ze::uniformDistribution<ze::uint8_t>(ZE_DETERMINISTIC);

  size_t width = 123;
  size_t height = 324;
  ze::ImageRaw8uC1 im(width,height);
  std::uint8_t min_val = std::numeric_limits<std::uint8_t>::max();
  std::uint8_t max_val = std::numeric_limits<std::uint8_t>::lowest();
  for (size_t y=0; y<height; ++y)
  {
    for (size_t x=0; x<width; ++x)
    {
      std::uint8_t random_value = random_val();
      im[y][x] = random_value;
      min_val = ze::cu::min(min_val, random_value);
      max_val = ze::cu::max(max_val, random_value);
    }
  }

  IMP_CUDA_CHECK();
  ze::cu::ImageGpu8uC1 cu_im(im);
  IMP_CUDA_CHECK();
  ze::Pixel8uC1 min_pixel, max_pixel;
  IMP_CUDA_CHECK();
  ze::cu::minMax(cu_im, min_pixel, max_pixel);
  IMP_CUDA_CHECK();

  ASSERT_EQ(min_val, min_pixel);
  ASSERT_EQ(max_val, max_pixel);
}


TEST(IMPCuCoreTestSuite,minMaxTest_32fC1)
{
  // setup random number generator
  auto random_val = ze::uniformDistribution<float>(ZE_DETERMINISTIC);

  size_t width = 1250;
  size_t height = 325;
  ze::ImageRaw32fC1 im(width,height);
  float min_val = std::numeric_limits<float>::max();
  float max_val = std::numeric_limits<float>::lowest();
  for (size_t y=0; y<height; ++y)
  {
    for (size_t x=0; x<width; ++x)
    {
      float random_value = random_val();
      im[y][x] = random_value;
      min_val = ze::cu::min(min_val, random_value);
      max_val = ze::cu::max(max_val, random_value);

//      VLOG(1) << "random: [" << x << "][" << y << "]: " << random_value;
    }
  }

  VLOG(1) << "numeric:  min, max: " << std::numeric_limits<float>::lowest() << " " << std::numeric_limits<float>::max();
  VLOG(1) << "numeric2: min, max: " << FLT_MIN << " " << FLT_MAX;
  VLOG(1) << "CPU min, max: " << min_val << " " << max_val;

  IMP_CUDA_CHECK();
  ze::cu::ImageGpu32fC1 cu_im(im);
  IMP_CUDA_CHECK();
  ze::Pixel32fC1 min_pixel, max_pixel;
  IMP_CUDA_CHECK();
  ze::cu::minMax(cu_im, min_pixel, max_pixel);
  IMP_CUDA_CHECK();

  VLOG(1) << "GPU min, max: " << min_pixel << " " << max_pixel;


  ASSERT_EQ(min_val, min_pixel.x);
  ASSERT_EQ(max_val, max_pixel.x);
}
