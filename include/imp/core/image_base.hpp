#ifndef IMP_IMAGE_BASE_HPP
#define IMP_IMAGE_BASE_HPP

#include <memory>

#include <imp/core/pixel_enums.hpp>
#include <imp/core/size.hpp>
#include <imp/core/roi.hpp>
#include <imp/core/types.hpp>

namespace imp {

/**
 * @brief The ImageBase class is the base class of all our image representations.
 *
 * It defines the common interface that must be implemented for IMP images
 *
 */
class ImageBase
{
public:
  using Ptr = std::shared_ptr<ImageBase>;

protected:
  ImageBase() = delete;

  ImageBase(PixelType pixel_type, PixelOrder pixel_order = imp::PixelOrder::undefined)
    : pixel_type_(pixel_type)
    , pixel_order_(pixel_order)
    , size_(0,0)
    , roi_(0,0,0,0)
  {
  }

  ImageBase(std::uint32_t width, std::uint32_t height,
            PixelType pixel_type,
            PixelOrder pixel_order = imp::PixelOrder::undefined)
    : pixel_type_(pixel_type)
    , pixel_order_(pixel_order)
    , size_(width, height)
    , roi_(size_)
  {
  }

  ImageBase(const Size2u &size, PixelType pixel_type,
            PixelOrder pixel_order = imp::PixelOrder::undefined)
    : pixel_type_(pixel_type)
    , pixel_order_(pixel_order)
    , size_(size)
    , roi_(size)
  {
  }

  ImageBase(const ImageBase &from)
    : pixel_type_(from.pixelType())
    , pixel_order_(from.pixelOrder())
    , size_(from.size_)
    , roi_(from.roi_)
  {
  }

public:
  virtual ~ImageBase() = default;

  ImageBase& operator= (const ImageBase &from)
  {
    // TODO == operator
    this->pixel_type_ = from.pixel_type_;
    this->pixel_order_ = from.pixel_order_;
    this->size_ = from.size_;
    this->roi_ = from.roi_;
    return *this;
  }

  virtual void setRoi(const imp::Roi2u& roi)
  {
    roi_ = roi;
  }

  /** Returns the element types. */
  PixelType pixelType() const
  {
    return pixel_type_;
  }

  /** Returns the pixel's channel order. */
  PixelOrder pixelOrder() const
  {
    return pixel_order_;
  }

  Size2u size() const
  {
    return size_;
  }

  Roi2u roi() const
  {
    return roi_;
  }

  std::uint32_t width() const
  {
    return size_[0];
  }

  std::uint32_t height() const
  {
    return size_[1];
  }

  std::uint8_t nChannels() const
  {
    switch (pixel_type_)
    {
    case PixelType::i8uC1:
    case PixelType::i16uC1:
    case PixelType::i32uC1:
    case PixelType::i32sC1:
    case PixelType::i32fC1:
      return 1;
    case PixelType::i8uC2:
    case PixelType::i16uC2:
    case PixelType::i32uC2:
    case PixelType::i32sC2:
    case PixelType::i32fC2:
      return 2;
    case PixelType::i8uC3:
    case PixelType::i16uC3:
    case PixelType::i32uC3:
    case PixelType::i32sC3:
    case PixelType::i32fC3:
      return 3;
    case PixelType::i8uC4:
    case PixelType::i16uC4:
    case PixelType::i32uC4:
    case PixelType::i32sC4:
    case PixelType::i32fC4:
      return 4;
    default:
      return 0;
    }
  }

  /** Returns the number of pixels in the image. */
  size_t numel() const
  {
    return (size_[0]*size_[1]);
  }

  /** Returns the total amount of bytes saved in the data buffer. */
  virtual size_t bytes() const
  {
    return this->height() * this->pitch();
  }

  /** Returns the length of a row (not including the padding!) in bytes. */
  virtual size_t rowBytes() const = 0;

  /** Returns the distance in bytes between starts of consecutive rows. */
  virtual size_t pitch() const = 0;

  /** Returns the distnace in pixels between starts of consecutive rows. */
  virtual size_t stride() const = 0;

  /** Returns the bit depth of the data pointer. */
  virtual std::uint8_t bitDepth() const = 0;

  /** Returns flag if the image data resides on the device/GPU (TRUE) or host/GPU (FALSE) */
  virtual bool isGpuMemory() const = 0;

  friend std::ostream& operator<<(std::ostream &os, const ImageBase& image);

protected:
  PixelType pixel_type_;
  PixelOrder pixel_order_;
  Size2u size_;
  Roi2u roi_;
};

inline std::ostream& operator<<(std::ostream &os, const ImageBase& image)
{
  os << "size: " << image.width() << "x" << image.height()
     << "; roi=(" << image.roi().x() << "," << image.roi().y()
     << "+" << image.roi().width() << "+" << image.roi().height() << ")"
     << "; stride: " << image.stride() << "; pitch: " << image.pitch()
     << "; bitDepth: " << (int)image.bitDepth();
  if (image.isGpuMemory())
    os << "; (gpu)";
  else
    os << "; (cpu)";

  return os;
}


// convenience typedefs
using ImageBasePtr = std::shared_ptr<ImageBase>;

} // namespace imp

#endif // IMP_IMAGE_BASE_HPP
