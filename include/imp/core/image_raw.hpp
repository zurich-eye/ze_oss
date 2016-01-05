#ifndef IMP_IMAGE_RAW_HPP
#define IMP_IMAGE_RAW_HPP

#include <memory>
#include <algorithm>

#include <imp/core/image.hpp>
#include <imp/core/memory_storage.hpp>

namespace imp {

/**
 * @brief The ImageRaw class is an image (surprise) holding raw memory
 *
 * The ImageRaw memory can be used to allocate raw memory of a given size, or
 * take external memory and hold a reference to that. Note that when external
 * memory is given to the class, the memory won't be managed! You have to take
 * care about the memory deletion. Instead when you let the class itself allocate
 * the memory it will take care of freeing the memory again. In addition the allocation
 * takes care about memory address alignment (default: 32-byte) for the beginning of
 * every row.
 *
 * The template parameters are as follows:
 *   - Pixel: The pixel's memory representation (e.g. imp::Pixel8uC1 for single-channel unsigned 8-bit images)
 *   - pixel_type: The internal enum for specifying the pixel's type more specificly
 */
template<typename Pixel, imp::PixelType pixel_type>
class ImageRaw : public imp::Image<Pixel, pixel_type>
{
public:
  using Ptr = typename std::shared_ptr<ImageRaw<Pixel,pixel_type>>;

  using Base = Image<Pixel, pixel_type>;
  using Memory = imp::MemoryStorage<Pixel>;
  using Deallocator = imp::MemoryDeallocator<Pixel>;

public:
  ImageRaw() = default;
  virtual ~ImageRaw() = default;

  /**
   * @brief ImageRaw construcs an image of given size \a width x \a height
   */
  ImageRaw(std::uint32_t width, std::uint32_t height);

  /**
   * @brief ImageRaw construcs an image of given \a size
   */
  ImageRaw(const imp::Size2u& size);

  /**
   * @brief ImageRaw copy constructs an image from the given image \a from
   */
  ImageRaw(const ImageRaw& from);

  /**
   * @brief ImageRaw copy constructs an arbitrary base image \a from (not necessarily am \a ImageRaw)
   */
  ImageRaw(const Base& from);

  /**
   * @brief ImageRaw constructs an image with the given data (copied or refererenced!)
   * @param data Pointer to the image data.
   * @param width Image width.
   * @param height Image height.
   * @param pitch Length of a row in bytes (including padding).
   * @param use_ext_data_pointer Flagg if the image should be copied (true) or if the data is just safed as 'reference' (false)
   */
  ImageRaw(Pixel* data, std::uint32_t width, std::uint32_t height,
           size_t pitch, bool use_ext_data_pointer = false);

  /**
   * @brief ImageRaw constructs an image with the given data shared with the given tracked object
   * @param data Pointer to the image data.
   * @param width Image width.
   * @param height Image height.
   * @param pitch Length of a row in bytes (including padding).
   * @param tracked Tracked object that shares the given image data
   * @note we assume that the tracked object takes care about memory deallocations
   */
  ImageRaw(Pixel* data, std::uint32_t width, std::uint32_t height,
           size_t pitch, const std::shared_ptr<void const>& tracked);


  /** Returns a pointer to the pixel data.
   * The pointer can be offset to position \a (ox/oy).
   * @param[in] ox Horizontal/Column offset of the pointer array.
   * @param[in] oy Vertical/Row offset of the pointer array.
   * @return Pointer to the pixel array.
   */
  virtual Pixel* data(std::uint32_t ox = 0, std::uint32_t oy = 0) override;
  virtual const Pixel* data(std::uint32_t ox = 0, std::uint32_t oy = 0) const override;

  /** Returns the distance in bytes between starts of consecutive rows. */
  virtual size_t pitch() const override { return pitch_; }

  /** Returns flag if the image data resides on the device/GPU (TRUE) or host/GPU (FALSE) */
  virtual bool isGpuMemory() const override { return false; }

protected:
  std::unique_ptr<Pixel, Deallocator> data_; //!< the actual image data
  size_t pitch_ = 0; //!< Row alignment in bytes.
  std::shared_ptr<void const> tracked_ = nullptr; //!< tracked object to share memory
};

//-----------------------------------------------------------------------------
// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef ImageRaw<imp::Pixel8uC1, imp::PixelType::i8uC1> ImageRaw8uC1;
typedef ImageRaw<imp::Pixel8uC2, imp::PixelType::i8uC2> ImageRaw8uC2;
typedef ImageRaw<imp::Pixel8uC3, imp::PixelType::i8uC3> ImageRaw8uC3;
typedef ImageRaw<imp::Pixel8uC4, imp::PixelType::i8uC4> ImageRaw8uC4;

typedef ImageRaw<imp::Pixel16uC1, imp::PixelType::i16uC1> ImageRaw16uC1;
typedef ImageRaw<imp::Pixel16uC2, imp::PixelType::i16uC2> ImageRaw16uC2;
typedef ImageRaw<imp::Pixel16uC3, imp::PixelType::i16uC3> ImageRaw16uC3;
typedef ImageRaw<imp::Pixel16uC4, imp::PixelType::i16uC4> ImageRaw16uC4;

typedef ImageRaw<imp::Pixel32sC1, imp::PixelType::i32sC1> ImageRaw32sC1;
typedef ImageRaw<imp::Pixel32sC2, imp::PixelType::i32sC2> ImageRaw32sC2;
typedef ImageRaw<imp::Pixel32sC3, imp::PixelType::i32sC3> ImageRaw32sC3;
typedef ImageRaw<imp::Pixel32sC4, imp::PixelType::i32sC4> ImageRaw32sC4;

typedef ImageRaw<imp::Pixel32fC1, imp::PixelType::i32fC1> ImageRaw32fC1;
typedef ImageRaw<imp::Pixel32fC2, imp::PixelType::i32fC2> ImageRaw32fC2;
typedef ImageRaw<imp::Pixel32fC3, imp::PixelType::i32fC3> ImageRaw32fC3;
typedef ImageRaw<imp::Pixel32fC4, imp::PixelType::i32fC4> ImageRaw32fC4;

// shared pointers
template <typename Pixel, imp::PixelType pixel_type>
using ImageRawPtr = typename ImageRaw<Pixel,pixel_type>::Ptr;

template <typename Pixel, imp::PixelType pixel_type>
using ImageRawConstPtr = typename ImageRaw<Pixel,pixel_type>::ConstPtr;


} // namespace imp


#endif // IMP_IMAGE_RAW_HPP
