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
#include <iostream>

#include <imp/cu_core/cu_image_gpu.cuh>

template<typename _Pixel, imp::PixelType _pixel_type>
struct TypePair
{
  using Pixel = _Pixel;
  //typedef typename _pixel_type pixel_type;
  using pixel_type = _pixel_type;
};

template <class T>
class ImageGpuTest : public ::testing::Test
{
protected:
  ImageGpuTest() :
    im_(width_,height_)
  {
  }

  size_t pixel_size_ = sizeof(T::Pixel);
  size_t pixel_bit_depth_ = 8*sizeof(T::Pixel);

  size_t width_ = 256;
  size_t height_ = 256;
  imp::cu::ImageGpu<typename T::Pixel, T::pixel_type> im_;
};

// The list of types we want to test.
typedef testing::Types<
TypePair<imp::Pixel8uC1, imp::PixelType::i8uC1>,
TypePair<imp::Pixel8uC2, imp::PixelType::i8uC2>,
TypePair<imp::Pixel8uC3, imp::PixelType::i8uC3>,
TypePair<imp::Pixel8uC4, imp::PixelType::i8uC4>,

TypePair<imp::Pixel16uC1, imp::PixelType::i16uC1>,
TypePair<imp::Pixel16uC2, imp::PixelType::i16uC2>,
TypePair<imp::Pixel16uC3, imp::PixelType::i16uC3>,
TypePair<imp::Pixel16uC4, imp::PixelType::i16uC4>,

TypePair<imp::Pixel32sC1, imp::PixelType::i32sC1>,
TypePair<imp::Pixel32sC2, imp::PixelType::i32sC2>,
TypePair<imp::Pixel32sC3, imp::PixelType::i32sC3>,
TypePair<imp::Pixel32sC4, imp::PixelType::i32sC4>,

TypePair<imp::Pixel32fC1, imp::PixelType::i32fC1>,
TypePair<imp::Pixel32fC2, imp::PixelType::i32fC2>,
TypePair<imp::Pixel32fC3, imp::PixelType::i32fC3>,
TypePair<imp::Pixel32fC4, imp::PixelType::i32fC4>
> PixelTypePairs;

TYPED_TEST_CASE(ImageGpuTest, PixelTypePairs);


TYPED_TEST(ImageGpuTest, CheckWidth)
{
  ASSERT_EQ(this->width_, this->im_.width());
}

//TYPED_TEST(ImageGpuTest, CheckHeight)
//{
//  ASSERT_EQ(this->height_, this->im_.height());
//}

//TYPED_TEST(ImageGpuTest, CheckMemoryAlignment)
//{
//  ASSERT_EQ(0, (std::uintptr_t)reinterpret_cast<void*>(this->im_.data()) % 32);
//}

//TYPED_TEST(ImageGpuTest, CheckNnumel)
//{
//  ASSERT_EQ(this->height_*this->width_, this->im_.numel());
//}

//TYPED_TEST(ImageGpuTest, CheckNumBytes)
//{
//  ASSERT_LE(this->height_*this->width_*this->pixel_size_, this->im_.bytes());
//}

//TYPED_TEST(ImageGpuTest, CheckPixelBitDepth)
//{
//  ASSERT_EQ(this->pixel_bit_depth_, this->im_.bitDepth());
//}

//TYPED_TEST(ImageGpuTest, ReturnsTrueForGpuMemory)
//{
//  ASSERT_TRUE(this->im_.isGpuMemory());
//}
