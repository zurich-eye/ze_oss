#ifndef IMP_IMAGE_HPP
#define IMP_IMAGE_HPP

#include <cstdint>

#include <imp/core/macros.hpp>
#include <imp/core/image_base.hpp>
#include <imp/core/exception.hpp>
#include <imp/core/pixel.hpp>

namespace imp {

template<typename Pixel, typename imp::PixelType pixel_type>
class Image : public ImageBase
{
public:
  using Ptr = typename std::shared_ptr<Image<Pixel,pixel_type>>;
  using pixel_t = Pixel;

protected:
  Image(imp::PixelOrder pixel_order = imp::PixelOrder::undefined)
    : ImageBase(pixel_type, pixel_order)
  { ; }

  Image(std::uint32_t width, std::uint32_t height,
        PixelOrder pixel_order = imp::PixelOrder::undefined)
    : ImageBase(width, height, pixel_type, pixel_order)
  { ; }

  Image(const imp::Size2u &size,
        imp::PixelOrder pixel_order = imp::PixelOrder::undefined)
    : ImageBase(size, pixel_type, pixel_order)
  { ; }

  Image(const Image& from) = default;

public:
  Image() = delete;
  virtual ~Image() = default;

  /** Returns a pointer to the pixel data.
   * The pointer can be offset to position \a (ox/oy).
   * @param[in] ox Horizontal offset of the pointer array.
   * @param[in] oy Vertical offset of the pointer array.
   * @return Pointer to the pixel array.
   */
  virtual Pixel* data(std::uint32_t ox = 0, std::uint32_t oy = 0) = 0;
  virtual const Pixel* data(std::uint32_t ox = 0, std::uint32_t oy = 0) const = 0;

  /** Get Pixel value at position x,y. */
  Pixel pixel(std::uint32_t x, std::uint32_t y) const
  {
    return *data(x, y);
  }

  /** Get Pixel value at position x,y. */
  Pixel& pixel(std::uint32_t x, std::uint32_t y)
  {
    return *data(x, y);
  }

  /** Get Pixel value at position x,y. */
  Pixel operator()(std::uint32_t x, std::uint32_t y) const
  {
    return *data(x, y);
  }

  /** Get Pointer to beginning of row \a row (y index).
   * This enables the usage of [y][x] operator.
   */
  Pixel* operator[] (std::uint32_t row)
  {
    return data(0,row);
  }
  const Pixel* operator[] (std::uint32_t row) const
  {
    return data(0,row);
  }


  /**
   * @brief setValue Sets image data to the specified \a value.
   * @param value Value to be set to the whole image data.
   */
  virtual void setValue(const Pixel& value)
  {
    if (this->bytes() == this->pitch()*this->height())
    {
      std::fill(this->data(), this->data()+this->stride()*this->height(), value);
    }
    else
    {
      for (std::uint32_t y=0; y<this->height(); ++y)
      {
        for (std::uint32_t x=0; x<this->width(); ++x)
        {
          this->data()[y*this->stride()+x] = value;
        }
      }
    }
  }

  /**
   * @brief copyTo copies the internal image data to another class instance
   * @param dst Image class that will receive this image's data.
   */
  virtual void copyTo(Image& dst) const
  {
    if (this->width() != dst.width() || this->height() != dst.height())
    {
      throw imp::Exception("Copying failed: Image size differs.", __FILE__, __FUNCTION__, __LINE__);
    }

    // check if dst image is on the gpu and the src image is not so we can
    // use the copyFrom functionality from the dst image as the Image class
    // doesn't know anything about gpu memory (poor thing)
    if (dst.isGpuMemory())
    {
      dst.copyFrom(*this);
    }
    else if (this->bytes() == dst.bytes())
    {
      std::copy(this->data(), this->data()+this->stride()*this->height(), dst.data());
    }
    else
    {
      for (std::uint32_t y=0; y<this->height(); ++y)
      {
        for (std::uint32_t x=0; x<this->width(); ++x)
        {
          dst[y][x] = this->pixel(x,y);
        }
      }
    }
  }

  /**
   * @brief copyFrom copies the image data from another class instance to this image
   * @param from Image class providing the image data.
   */
  virtual void copyFrom(const Image& from)
  {
    if (this->size()!= from.size())
    {
      throw imp::Exception("Copying failed: Image sizes differ.", __FILE__, __FUNCTION__, __LINE__);
    }

    if (from.isGpuMemory())
    {
      from.copyTo(*this);
    }
    else if (this->bytes() == from.bytes())
    {
      std::copy(from.data(), from.data()+from.stride()*from.height(), this->data());
    }
    else
    {
      for (std::uint32_t y=0; y<this->height(); ++y)
      {
        for (std::uint32_t x=0; x<this->width(); ++x)
        {
          (*this)[y][x] = from.pixel(y,x);
        }
      }
    }
  }

  /** Returns the length of a row (not including the padding!) in bytes. */
  virtual size_t rowBytes() const
  {
    return this->width() * sizeof(Pixel);
  }

  /** Returns the distnace in pixels between starts of consecutive rows. */
  virtual size_t stride() const override
  {
    return this->pitch()/sizeof(Pixel);
  }

  /** Returns the bit depth of the data pointer. */
  virtual std::uint8_t bitDepth() const override
  {
    return 8*sizeof(Pixel);
  }
};

//-----------------------------------------------------------------------------
// convenience typedefs
typedef Image<imp::Pixel8uC1, imp::PixelType::i8uC1> Image8uC1;
typedef Image<imp::Pixel8uC2, imp::PixelType::i8uC2> Image8uC2;
typedef Image<imp::Pixel8uC3, imp::PixelType::i8uC3> Image8uC3;
typedef Image<imp::Pixel8uC4, imp::PixelType::i8uC4> Image8uC4;

typedef Image<imp::Pixel16uC1, imp::PixelType::i16uC1> Image16uC1;
typedef Image<imp::Pixel16uC2, imp::PixelType::i16uC2> Image16uC2;
typedef Image<imp::Pixel16uC3, imp::PixelType::i16uC3> Image16uC3;
typedef Image<imp::Pixel16uC4, imp::PixelType::i16uC4> Image16uC4;

typedef Image<imp::Pixel32sC1, imp::PixelType::i32sC1> Image32sC1;
typedef Image<imp::Pixel32sC2, imp::PixelType::i32sC2> Image32sC2;
typedef Image<imp::Pixel32sC3, imp::PixelType::i32sC3> Image32sC3;
typedef Image<imp::Pixel32sC4, imp::PixelType::i32sC4> Image32sC4;

typedef Image<imp::Pixel32fC1, imp::PixelType::i32fC1> Image32fC1;
typedef Image<imp::Pixel32fC2, imp::PixelType::i32fC2> Image32fC2;
typedef Image<imp::Pixel32fC3, imp::PixelType::i32fC3> Image32fC3;
typedef Image<imp::Pixel32fC4, imp::PixelType::i32fC4> Image32fC4;

// convenience typedefs
template<typename Pixel, imp::PixelType pixel_type>
using ImagePtr = typename std::shared_ptr<Image<Pixel,pixel_type>>;



} // namespace imp

#endif // IMP_IMAGE_HPP
