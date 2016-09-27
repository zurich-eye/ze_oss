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
#include <sstream>
#include <string>

#include <ze/common/benchmark.hpp>
#include <ze/common/random.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/types.hpp>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_math.cuh>
#include <imp/core/image_raw.hpp>

template <typename Pixel>
class CuWeightedSumTestFixture : public ::testing::Test
{
protected:
  CuWeightedSumTestFixture()
    : image1_(size_)
    , image2_(size_)
    , dst_(size_)
    , cu_image1_(size_)
    , cu_image2_(size_)
    , cu_dst_(size_)
  {
    auto random_val_generator = ze::uniformDistribution<typename Pixel::T>(ZE_DETERMINISTIC);
    val1_ = random_val_generator();
    val2_ = random_val_generator();
    auto float_random_val_generator = ze::uniformDistribution<float>(ZE_DETERMINISTIC);
    weight1_ = float_random_val_generator();
    weight2_ = float_random_val_generator();

    if (std::is_integral<typename Pixel::T>::value)
    {
      VLOG(100) << "given Pixel type is integral";
      dst_val_ = static_cast<Pixel>(val1_*weight1_ + val2_*weight2_ + 0.5f);
    }
    else
    {
      VLOG(100) << "given Pixel type is non-integral";
      dst_val_ = val1_*weight1_ + val2_*weight2_;
    }

    image1_.setValue(val1_);
    image2_.setValue(val2_);

    VLOG(1) << "val1: '" << val1_ << "', val2: '" << val2_ << "'";
    VLOG(1) << "weight1: '" << weight1_ << "', weight2: '" << weight2_ << "'";
    VLOG(1) << "dst_val: '" << dst_val_ << "'";

    VLOG(1) << "image1 | size: " << image1_.size() << ", roi: " << image1_.roi()
            << " value(10,10): " << image1_.pixel(10,10);
    VLOG(1) << "image2 | size: " << image2_.size() << ", roi: " << image2_.roi()
            << " value(10,10): " << image2_.pixel(10,10);
    cu_image1_.copyFrom(image1_);
    cu_image2_.copyFrom(image2_);
  }

  void setRoi()
  {
    image1_.setRoi(roi_);
    image2_.setRoi(roi_);
    cu_image1_.setRoi(roi_);
    cu_image2_.setRoi(roi_);
    dst_.setRoi(roi_);
    cu_dst_.setRoi(roi_);
  }


  void weightedSum()
  {
    ze::cu::weightedSum(cu_dst_, cu_image1_, weight1_, cu_image2_, weight2_);
    cu_dst_.copyTo(dst_);
    VLOG(1) << "dst | size: " << dst_.size() << ", roi: " << dst_.roi()
            << " value(10,10): " << dst_.pixel(10,10);
  }

protected:
  //! @todo random size and roi!
  ze::Size2u size_{511u,512u};
  ze::Roi2u roi_{128,128,256,256};
  ze::ImageRaw<Pixel> image1_;
  ze::ImageRaw<Pixel> image2_;
  ze::ImageRaw<Pixel> dst_;
  ze::cu::ImageGpu<Pixel> cu_image1_;
  ze::cu::ImageGpu<Pixel> cu_image2_;
  ze::cu::ImageGpu<Pixel> cu_dst_;
  Pixel val1_;
  Pixel val2_;
  Pixel dst_val_;
  float weight1_;
  float weight2_;
};


// The list of types we want to test.
typedef testing::Types<
ze::Pixel8uC1, ze::Pixel32fC1
> PixelTypes;

TYPED_TEST_CASE(CuWeightedSumTestFixture, PixelTypes);

TYPED_TEST(CuWeightedSumTestFixture, testweightedSum)
{
  using namespace ze::cu;

  this->weightedSum();
  for (uint32_t y=0; y<this->dst_.height(); ++y)
  {
    for (uint32_t x=0; x<this->dst_.width(); ++x)
    {
      CHECK_EQ(this->dst_val_, this->dst_.pixel(x,y));
    }
  }
}

TYPED_TEST(CuWeightedSumTestFixture, testweightedSumRoi)
{
  using namespace ze::cu;

  this->setRoi();
  this->weightedSum();
  for (uint32_t y=this->roi_.y(); y<this->roi_.y()+this->roi_.height(); ++y)
  {
    for (uint32_t x=this->roi_.x(); x<this->roi_.x()+this->roi_.width(); ++x)
    {
      CHECK_EQ(this->dst_val_, this->dst_.pixel(x,y));
    }
  }
}

ZE_UNITTEST_ENTRYPOINT
