#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>
#include <functional>
#include <limits>
#include <type_traits>

#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_linearmemory.cuh>
#include <ze/common/random.hpp>
#include <ze/common/test_utils.hpp>

template <typename Pixel>
class CuLinearMemoryTest : public ::testing::Test
{
 protected:
  CuLinearMemoryTest()
    : linmem_(numel_)
    , linmem_copy_(numel_)
    , cu_linmem_(numel_)
    , cu_linmem_init0_(numel_)
    , cu_linmem_init_rand_(numel_)
    , linmem_init0_cp_(numel_)
    , linmem_init_rand_cp_(numel_)
    , cu_roi_linmem_(roi_.length())
    , roi_linmem_copy_(roi_.length())
  {
    using T = typename Pixel::T;
    auto random_val_generator = ze::uniformDistribution<T>(ZE_DETERMINISTIC);

    for (size_t i=0; i<this->numel_; ++i)
    {
      linmem_[i] = random_val_generator();
    }

    cu_linmem_init_rand_pixel_val_ = Pixel(random_val_generator());
    cu_linmem_roi_init_rand_pixel_val_ = Pixel(random_val_generator());
  }

  void roundtripCopy()
  {
    cu_linmem_.copyFrom(linmem_);
    IMP_CUDA_CHECK();
    cu_linmem_.copyTo(linmem_copy_);
    IMP_CUDA_CHECK();
  }

  void initZeroAndCopy()
  {
    cu_linmem_init0_.setValue(cu_linmem_init0_pixel_val_);
    IMP_CUDA_CHECK();
    cu_linmem_init0_.copyTo(linmem_init0_cp_);
    IMP_CUDA_CHECK();
  }

  void setRandomValueAndCopy()
  {
    cu_linmem_init_rand_.setValue(cu_linmem_init_rand_pixel_val_);
    IMP_CUDA_CHECK();
    cu_linmem_init_rand_.copyTo(linmem_init_rand_cp_);
    IMP_CUDA_CHECK();
  }

  void setRoi()
  {
    cu_linmem_.setRoi(roi_);
    IMP_CUDA_CHECK();
  }

  void setRoiRandomValueAndCopy()
  {
    cu_linmem_init_rand_.setValue(cu_linmem_init_rand_pixel_val_);
    IMP_CUDA_CHECK();
    cu_linmem_init_rand_.setRoi(roi_);
    cu_linmem_init_rand_.setValue(cu_linmem_roi_init_rand_pixel_val_);
    cu_linmem_init_rand_.resetRoi();
    IMP_CUDA_CHECK();

    cu_linmem_init_rand_.copyTo(linmem_init_rand_cp_);
    IMP_CUDA_CHECK();
  }

  void roiRoundtripCopy()
  {
    linmem_.setRoi(roi_);
    IMP_CUDA_CHECK();
    cu_roi_linmem_.copyFrom(linmem_);
    IMP_CUDA_CHECK();
    cu_roi_linmem_.copyTo(roi_linmem_copy_);
    IMP_CUDA_CHECK();
  }


  uint8_t pixel_size_ = sizeof(Pixel);
  size_t pixel_bit_depth_ = 8*sizeof(Pixel);

  size_t numel_ = 10000;
  ze::LinearMemory<Pixel> linmem_;
  ze::LinearMemory<Pixel> linmem_copy_;
  ze::cu::LinearMemory<Pixel> cu_linmem_;
  Pixel cu_linmem_init0_pixel_val_ = Pixel(0);
  ze::cu::LinearMemory<Pixel> cu_linmem_init0_;
  ze::cu::LinearMemory<Pixel> cu_linmem_init_rand_;
  Pixel cu_linmem_init_rand_pixel_val_;

  ze::Roi1u roi_ = ze::Roi1u(numel_/3, numel_/3);
  Pixel cu_linmem_roi_init_rand_pixel_val_;
  ze::LinearMemory<Pixel> linmem_init0_cp_;
  ze::LinearMemory<Pixel> linmem_init_rand_cp_;

  ze::cu::LinearMemory<Pixel> cu_roi_linmem_;
  ze::LinearMemory<Pixel> roi_linmem_copy_;
};

// The list of types we want to test.
typedef testing::Types<
ze::Pixel8uC1, ze::Pixel8uC2, ze::Pixel8uC3, ze::Pixel8uC4,
ze::Pixel16uC1, ze::Pixel16uC2, ze::Pixel16uC3, ze::Pixel16uC4,
ze::Pixel32sC1, ze::Pixel32sC2, ze::Pixel32sC3, ze::Pixel32sC4,
ze::Pixel32uC1, ze::Pixel32uC2, ze::Pixel32uC3, ze::Pixel32uC4,
ze::Pixel32fC1, ze::Pixel32fC2, ze::Pixel32fC3, ze::Pixel32fC4
> PixelTypes;

TYPED_TEST_CASE(CuLinearMemoryTest, PixelTypes);

// alignment check deleted on purpose as the alignment is handeled by cuda and we don't mess around with that

TYPED_TEST(CuLinearMemoryTest, CheckLength)
{
  ASSERT_EQ(this->numel_, this->cu_linmem_.length());
}

TYPED_TEST(CuLinearMemoryTest, CheckNumBytes)
{
  ASSERT_EQ(this->numel_*this->pixel_size_, this->cu_linmem_.bytes());
}

TYPED_TEST(CuLinearMemoryTest, CheckPixelBitDepth)
{
  ASSERT_EQ(this->pixel_bit_depth_, this->cu_linmem_.bitDepth());
}

TYPED_TEST(CuLinearMemoryTest, ReturnsTrueForNonGpuMemory)
{
  ASSERT_TRUE(this->cu_linmem_.isGpuMemory());
}

TYPED_TEST(CuLinearMemoryTest, CheckRoundTripCopy)
{
  this->roundtripCopy();
  for (size_t i=0; i<this->numel_; ++i) {
    ASSERT_EQ(this->linmem_[i], this->linmem_copy_[i]);
  }
}

TYPED_TEST(CuLinearMemoryTest, CheckInitZero)
{
  this->initZeroAndCopy();
  for (size_t i=0; i<this->numel_; ++i) {
    ASSERT_EQ(this->linmem_init0_cp_[i], this->cu_linmem_init0_pixel_val_);
  }
}

TYPED_TEST(CuLinearMemoryTest, CheckSetRandomValue)
{
  this->setRandomValueAndCopy();
  for (size_t i=0; i<this->numel_; ++i) {
    ASSERT_EQ(this->linmem_init_rand_cp_[i], this->cu_linmem_init_rand_pixel_val_);
  }
}

TYPED_TEST(CuLinearMemoryTest, CheckNoRoi)
{
  ASSERT_EQ(0, this->cu_linmem_.roi().x());
  ASSERT_EQ(this->numel_, this->cu_linmem_.roi().length());
}

TYPED_TEST(CuLinearMemoryTest, CheckRoi)
{
  this->setRoi();
  ASSERT_EQ(this->roi_.x(), this->cu_linmem_.roi().x());
  ASSERT_EQ(this->roi_.length(), this->cu_linmem_.roi().length());
}

TYPED_TEST(CuLinearMemoryTest, CheckRoiSetRandomValue)
{
  this->setRoiRandomValueAndCopy();
  for (size_t i=0; i<this->numel_; ++i) {
    if (i>=this->roi_.x() && i<(this->roi_.x()+this->roi_.length())) {
      ASSERT_EQ(this->linmem_init_rand_cp_[i], this->cu_linmem_roi_init_rand_pixel_val_);
    }
    else {
      ASSERT_EQ(this->linmem_init_rand_cp_[i], this->cu_linmem_init_rand_pixel_val_);
    }
  }
}

TYPED_TEST(CuLinearMemoryTest, CheckRoiRoundTripCopy)
{
  this->roiRoundtripCopy();
  for (size_t i=0; i<this->roi_.length(); ++i) {
    ASSERT_EQ(this->linmem_[i+this->roi_.x()], this->roi_linmem_copy_[i]);
  }
}
