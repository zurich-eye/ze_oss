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

#include <imp/core/image.hpp>
#include <imp/core/image_raw.hpp>
#include <ze/common/random.hpp>

namespace ze {
ImageRaw8uC1 generateRandomImage(size_t width, size_t height)
{
  auto random_val = uniformDistribution<uint8_t>(ZE_DETERMINISTIC);
  ImageRaw8uC1 img(width, height);
  for (size_t y = 0; y < img.height(); ++y)
  {
    for (size_t x = 0; x < img.width(); ++x)
    {
      uint8_t random_value = random_val();
      img[y][x] = random_value;
    }
  }
  return img;
}

} // ze namespace

TEST(IMPCoreTestSuite, testCopy8uC1)
{
  using namespace ze;
  ImageRaw8uC1 img = generateRandomImage(1024, 768);
  ImageRaw8uC1 img_copy(img.width(), img.height());
  img_copy.copyFrom(img);
  for (uint32_t x = 0; x < img_copy.width(); ++x)
  {
    for (uint32_t y = 0; y < img_copy.height(); ++y)
    {
      EXPECT_EQ(img(x, y), img_copy(x, y));
    }
  }
}

TEST(IMPCoreTestSuite, testCopy8uC1DifferentBytes)
{
  using namespace ze;

  constexpr uint32_t width{980};
  constexpr uint32_t padded_width{1024};
  constexpr uint32_t height{768};
  constexpr uint32_t padded_numel{padded_width * height};

  //! Allocate pitched memory and use external data pointer
  auto random_val = uniformDistribution<uint8_t>(ZE_DETERMINISTIC);
  Pixel8uC1 data_array[padded_numel];
  for (uint32_t i = 0; i < padded_numel; ++i)
  {
    data_array[i] = random_val();
  }
  ImageRaw8uC1 img(
        data_array,
        width, height,
        padded_width * sizeof(Pixel8uC1),
        true);

  //! Perform copy and test result
  ImageRaw8uC1 img_copy(img.width(), img.height());
  img_copy.copyFrom(img);
  for (uint32_t x = 0; x < img_copy.width(); ++x)
  {
    for (uint32_t y = 0; y < img_copy.height(); ++y)
    {
      EXPECT_EQ(img(x, y), img_copy(x, y));
    }
  }
}
