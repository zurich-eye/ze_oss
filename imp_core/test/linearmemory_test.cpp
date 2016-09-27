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
#include <functional>
#include <limits>
#include <type_traits>

#include <imp/core/linearmemory.hpp>
#include <ze/common/random.hpp>
#include <ze/common/test_utils.hpp>


template <typename Pixel>
class LinearMemoryTest : public ::testing::Test
{
 protected:
  LinearMemoryTest() :
    linmem_(numel_)
  {
    using T = typename Pixel::T;
    auto random_val_generator = ze::uniformDistribution<T>(ZE_DETERMINISTIC);

    T val1 = random_val_generator();
    T val2;
    do
    {
      val2 = random_val_generator();
    } while(std::fabs(val1-val2) < std::numeric_limits<T>::epsilon());

    pixel1_ = Pixel(val1);
    pixel2_ = Pixel(val2);
  }

  void setValue()
  {
    linmem_.setValue(pixel1_);
  }

  void setRoi()
  {
    linmem_.setRoi(roi_);
  }

  void setValueRoi()
  {
    this->setRoi();
    linmem_.setValue(pixel2_);
  }

  uint8_t pixel_size_ = sizeof(Pixel);
  size_t pixel_bit_depth_ = 8*sizeof(Pixel);

  size_t numel_ = 123;
  ze::Roi1u roi_ = ze::Roi1u(numel_/3, numel_/3);
  ze::LinearMemory<Pixel> linmem_;

  Pixel pixel1_;
  Pixel pixel2_;
};

// The list of types we want to test.
typedef testing::Types<
ze::Pixel8uC1, ze::Pixel8uC2, ze::Pixel8uC3, ze::Pixel8uC4,
ze::Pixel16uC1, ze::Pixel16uC2, ze::Pixel16uC3, ze::Pixel16uC4,
ze::Pixel32sC1, ze::Pixel32sC2, ze::Pixel32sC3, ze::Pixel32sC4,
ze::Pixel32uC1, ze::Pixel32uC2, ze::Pixel32uC3, ze::Pixel32uC4,
ze::Pixel32fC1, ze::Pixel32fC2, ze::Pixel32fC3, ze::Pixel32fC4> PixelTypes;

TYPED_TEST_CASE(LinearMemoryTest, PixelTypes);

TYPED_TEST(LinearMemoryTest, CheckMemforyAlignment)
{
  EXPECT_EQ(0u, (std::uintptr_t)reinterpret_cast<void*>(this->linmem_.data()) % 32);
}

TYPED_TEST(LinearMemoryTest, CheckLength)
{
  EXPECT_EQ(this->numel_, this->linmem_.length());
}

TYPED_TEST(LinearMemoryTest, CheckNoRoi)
{
  EXPECT_EQ(0u, this->linmem_.roi().x());
  EXPECT_EQ(this->numel_, this->linmem_.roi().length());
}

TYPED_TEST(LinearMemoryTest, CheckRoi)
{
  this->setRoi();
  EXPECT_EQ(this->roi_.x(), this->linmem_.roi().x());
  EXPECT_EQ(this->roi_.length(), this->linmem_.roi().length());
}

TYPED_TEST(LinearMemoryTest, CheckNumBytes)
{
  EXPECT_EQ(this->numel_*this->pixel_size_, this->linmem_.bytes());
}

TYPED_TEST(LinearMemoryTest, CheckNumRoiBytes)
{
  this->setRoi();
  EXPECT_EQ(this->roi_.length()*this->pixel_size_, this->linmem_.roiBytes());
}

TYPED_TEST(LinearMemoryTest, CheckPixelBitDepth)
{
  EXPECT_EQ(this->pixel_bit_depth_, this->linmem_.bitDepth());
}

TYPED_TEST(LinearMemoryTest, ReturnsFalseForNonGpuMemory)
{
  ASSERT_FALSE(this->linmem_.isGpuMemory());
}

TYPED_TEST(LinearMemoryTest, CheckValues)
{
  this->setValue();
  for (uint32_t i=0u; i<this->numel_; ++i)
  {
    EXPECT_EQ(this->linmem_[i], this->pixel1_);
  }
}

TYPED_TEST(LinearMemoryTest, CheckValuesInConstLinearMemory)
{
  this->setValue();
  const ze::LinearMemory<TypeParam> const_linmem(this->linmem_);
  for (uint32_t i=0u; i<this->numel_; ++i)
  {
    EXPECT_EQ(const_linmem[i], this->pixel1_);
  }
}


TYPED_TEST(LinearMemoryTest, CheckRoiValues)
{
  this->setValue();
  this->setValueRoi();

  for (uint32_t i=0u; i<this->numel_; ++i)
  {
    if (i>=this->roi_.x() && i<(this->roi_.x()+this->roi_.length()))
    {
      EXPECT_EQ(this->pixel2_, this->linmem_[i]);
    }
    else
    {
      EXPECT_EQ(this->pixel1_, this->linmem_[i]);
    }
  }
}

