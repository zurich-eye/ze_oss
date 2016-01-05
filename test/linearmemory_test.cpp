#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>
#include <random>
#include <functional>
#include <limits>
#include <type_traits>

#include <imp/core/linearmemory.hpp>

template<class T>
typename std::enable_if<std::is_integral<T>::value, std::function<T()> >::type
getRandomGenerator()
{
  std::default_random_engine generator(std::random_device{}());
  std::uniform_int_distribution<T> distribution(std::numeric_limits<T>::lowest(),
                                                std::numeric_limits<T>::max());
  auto random_val = std::bind(distribution, generator);
  return random_val;
}

template<class T>
typename std::enable_if<!std::is_integral<T>::value, std::function<T()> >::type
getRandomGenerator()
{
  std::default_random_engine generator(std::random_device{}());
  std::uniform_real_distribution<T> distribution(std::numeric_limits<T>::lowest(),
                                                 std::numeric_limits<T>::max());

  auto random_val = std::bind(distribution, generator);
  return random_val;
}


template <typename Pixel>
class LinearMemoryTest : public ::testing::Test
{
 protected:
  LinearMemoryTest() :
    linmem_(numel_)
  {
    using T = typename Pixel::T;
    auto random_val_generator = getRandomGenerator<T>();

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

  size_t pixel_size_ = sizeof(Pixel);
  size_t pixel_bit_depth_ = 8*sizeof(Pixel);

  size_t numel_ = 123;
  imp::Roi1u roi_ = imp::Roi1u(numel_/3, numel_/3);
  imp::LinearMemory<Pixel> linmem_;

  Pixel pixel1_;
  Pixel pixel2_;
};

// The list of types we want to test.
typedef testing::Types<
imp::Pixel8uC1, imp::Pixel8uC2, imp::Pixel8uC3, imp::Pixel8uC4,
imp::Pixel16uC1, imp::Pixel16uC2, imp::Pixel16uC3, imp::Pixel16uC4,
imp::Pixel32sC1, imp::Pixel32sC2, imp::Pixel32sC3, imp::Pixel32sC4,
imp::Pixel32uC1, imp::Pixel32uC2, imp::Pixel32uC3, imp::Pixel32uC4,
imp::Pixel32fC1, imp::Pixel32fC2, imp::Pixel32fC3, imp::Pixel32fC4> PixelTypes;

TYPED_TEST_CASE(LinearMemoryTest, PixelTypes);

TYPED_TEST(LinearMemoryTest, CheckMemforyAlignment)
{
  ASSERT_EQ(0, (std::uintptr_t)reinterpret_cast<void*>(this->linmem_.data()) % 32);
}

TYPED_TEST(LinearMemoryTest, CheckLength)
{
  ASSERT_EQ(this->numel_, this->linmem_.length());
}

TYPED_TEST(LinearMemoryTest, CheckNoRoi)
{
  ASSERT_EQ(0, this->linmem_.roi().x());
  ASSERT_EQ(this->numel_, this->linmem_.roi().length());
}

TYPED_TEST(LinearMemoryTest, CheckRoi)
{
  this->setRoi();
  ASSERT_EQ(this->roi_.x(), this->linmem_.roi().x());
  ASSERT_EQ(this->roi_.length(), this->linmem_.roi().length());
}

TYPED_TEST(LinearMemoryTest, CheckNumBytes)
{
  ASSERT_EQ(this->numel_*this->pixel_size_, this->linmem_.bytes());
}

TYPED_TEST(LinearMemoryTest, CheckNumRoiBytes)
{
  this->setRoi();
  ASSERT_EQ(this->roi_.length()*this->pixel_size_, this->linmem_.roiBytes());
}

TYPED_TEST(LinearMemoryTest, CheckPixelBitDepth)
{
  ASSERT_EQ(this->pixel_bit_depth_, this->linmem_.bitDepth());
}

TYPED_TEST(LinearMemoryTest, ReturnsFalseForNonGpuMemory)
{
  ASSERT_FALSE(this->linmem_.isGpuMemory());
}

TYPED_TEST(LinearMemoryTest, CheckValues)
{
  this->setValue();
  for (std::uint32_t i=0; i<this->numel_; ++i)
  {
    ASSERT_EQ(this->linmem_[i], this->pixel1_);
  }
}

TYPED_TEST(LinearMemoryTest, CheckValuesInConstLinearMemory)
{
  this->setValue();
  const imp::LinearMemory<TypeParam> const_linmem(this->linmem_);
  for (std::uint32_t i=0; i<this->numel_; ++i)
  {
    ASSERT_EQ(const_linmem[i], this->pixel1_);
  }
}


TYPED_TEST(LinearMemoryTest, CheckRoiValues)
{
  this->setValue();
  this->setValueRoi();

  for (std::uint32_t i=0; i<this->numel_; ++i)
  {
    if (i>=this->roi_.x() && i<(this->roi_.x()+this->roi_.length()))
    {
      ASSERT_EQ(this->pixel2_, this->linmem_[i]);
    }
    else
    {
      ASSERT_EQ(this->pixel1_, this->linmem_[i]);
    }
  }
}

