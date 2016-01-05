#ifndef IMP_CU_IMAGE_GPU_CUH
#define IMP_CU_IMAGE_GPU_CUH

#include <memory>
#include <algorithm>

#include <imp/core/types.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/cu_core/cu_exception.hpp>
#include <imp/cu_core/cu_memory_storage.cuh>
#include <imp/core/image.hpp>
#include <imp/cu_core/cu_pixel_conversion.hpp>


namespace imp {
namespace cu {

// forward declarations
class Texture2D;

/**
 * @brief The ImageGpu class is an image (surprise) holding raw memory
 *
 * The ImageGpu memory can be used to allocate raw memory of a given size, or
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
 *
 * @warning Be careful with 8-bit 3-channel GPU memory as the stride is not divisable by 3!
 */
template<typename Pixel, imp::PixelType pixel_type>
class ImageGpu : public imp::Image<Pixel, pixel_type>
{
public:
  using Ptr = typename std::shared_ptr<ImageGpu<Pixel,pixel_type>>;
  using UPtr = typename std::unique_ptr<ImageGpu<Pixel,pixel_type>>;

  using Base = Image<Pixel, pixel_type>;
  using Memory = imp::cu::MemoryStorage<Pixel>;
  using Deallocator = imp::cu::MemoryDeallocator<Pixel>;

public:
  ImageGpu() = delete;
  virtual ~ImageGpu();/* = default;*/

  /**
   * @brief ImageGpu construcs an image of given \a size
   */
  ImageGpu(const imp::Size2u& size);

  /**
   * @brief ImageGpu construcs an image of given size \a width x \a height
   */
  ImageGpu(std::uint32_t width, std::uint32_t height)
    : ImageGpu(imp::Size2u(width,height)) {;}

  /**
   * @brief ImageGpu copy constructs an image from the given image \a from
   */
  ImageGpu(const ImageGpu& from);

  /**
   * @brief ImageGpu copy construcs a GPU image from an arbitrary base image \a from (not necessarily am \a ImageGpu)
   */
  ImageGpu(const Base& from);

  /**
   * @brief ImageGpu copy constructs a GPU image from the 8-bit (cpu) image \a from
   */
  //ImageGpu(const Image8uC3& from);

  /**
   * @brief ImageGpu constructs an image with the given data (copied or refererenced!)
   * @param data Pointer to the image data.
   * @param width Image width.
   * @param height Image height.
   * @param pitch Length of a row in bytes (including padding).
   * @param use_ext_data_pointer Flagg if the image should be copied (true) or if the data is just safed as 'reference' (false)
   */
//  ImageGpu(Pixel* data, std::uint32_t width, std::uint32_t height,
//           size_t pitch, bool use_ext_data_pointer = false);

  /** sets a region of interest */
  virtual void setRoi(const imp::Roi2u& roi) override;

  /**
   * @brief copyTo copies the internal image data to another class instance
   * @param dst Image class that will receive this image's data.
   */
  virtual void copyTo(Base& dst) const override;

  /**
   * @brief copyFrom copies the image data from another class instance to this image
   * @param from Image class providing the image data.
   */
  virtual void copyFrom(const Base& from) override;

  /** Returns a pointer to the pixel data.
   * The pointer can be offset to position \a (ox/oy).
   * @param[in] ox Horizontal/Column offset of the pointer array.
   * @param[in] oy Vertical/Row offset of the pointer array.
   * @return Pointer to the pixel array.
   */
  virtual Pixel* data(std::uint32_t ox = 0, std::uint32_t oy = 0) override;
  virtual const Pixel* data(std::uint32_t ox = 0, std::uint32_t oy = 0) const override;

  /** Returns a cuda vector* that is pointing to the beginning for the data buffer.
   * @note this is mainly for convenience when calling cuda functions / kernels.
   */
  auto cuData() -> decltype(imp::cu::toCudaVectorType(this->data()));
  auto cuData() const -> decltype(imp::cu::toConstCudaVectorType(this->data()));


  /**
   * @brief setValue Sets image data to the specified \a value.
   * @param value Value to be set to the whole image data.
   * @note @todo (MWE) TBD: region-of-interest is considered
   */
  virtual void setValue(const Pixel& value) override;

  /** Returns the distance in bytes between starts of consecutive rows. */
  virtual size_t pitch() const override { return pitch_; }

  /** Returns flag if the image data resides on the device/GPU (TRUE) or host/GPU (FALSE) */
  virtual bool isGpuMemory() const override { return true; }

  /** Returns a data structure to operate within a cuda kernel (does not copy any memory!). */
//  std::unique_ptr<GpuData2D<Pixel>> gpuData() { return gpu_data_; }

  /** Returns the channel descriptor for Cuda's texture memory. */
  inline cudaChannelFormatDesc channelFormatDesc() const { return channel_format_desc_; }

  /** Returns a cuda texture object. */
  std::shared_ptr<Texture2D> genTexture(
      bool normalized_coords = false,
      cudaTextureFilterMode filter_mode = cudaFilterModePoint,
      cudaTextureAddressMode address_mode = cudaAddressModeClamp,
      cudaTextureReadMode read_mode = cudaReadModeElementType) const;

  // operators
  /** Pixel-wise multiplication. */
//  template<typename T>
//  ImageGpu<Pixel, pixel_type>& operator*=(const T& rhs);

  ImageGpu<Pixel, pixel_type>& operator*=(const Pixel& rhs);


protected:
  std::unique_ptr<Pixel, Deallocator> data_; //!< the actual image data
  size_t pitch_ = 0; //!< Row alignment in bytes.

private:
  cudaChannelFormatDesc channel_format_desc_;

  //std::unique_ptr<GpuData2D<Pixel>> gpu_data_; //!< data collection that can be directly used within a kernel.
//  GpuData2D<Pixel>* gpu_data_;
};

//-----------------------------------------------------------------------------
// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef ImageGpu<imp::Pixel8uC1, imp::PixelType::i8uC1> ImageGpu8uC1;
typedef ImageGpu<imp::Pixel8uC2, imp::PixelType::i8uC2> ImageGpu8uC2;
typedef ImageGpu<imp::Pixel8uC3, imp::PixelType::i8uC3> ImageGpu8uC3;
typedef ImageGpu<imp::Pixel8uC4, imp::PixelType::i8uC4> ImageGpu8uC4;

typedef ImageGpu<imp::Pixel16uC1, imp::PixelType::i16uC1> ImageGpu16uC1;
typedef ImageGpu<imp::Pixel16uC2, imp::PixelType::i16uC2> ImageGpu16uC2;
typedef ImageGpu<imp::Pixel16uC3, imp::PixelType::i16uC3> ImageGpu16uC3;
typedef ImageGpu<imp::Pixel16uC4, imp::PixelType::i16uC4> ImageGpu16uC4;

typedef ImageGpu<imp::Pixel32sC1, imp::PixelType::i32sC1> ImageGpu32sC1;
typedef ImageGpu<imp::Pixel32sC2, imp::PixelType::i32sC2> ImageGpu32sC2;
typedef ImageGpu<imp::Pixel32sC3, imp::PixelType::i32sC3> ImageGpu32sC3;
typedef ImageGpu<imp::Pixel32sC4, imp::PixelType::i32sC4> ImageGpu32sC4;

typedef ImageGpu<imp::Pixel32fC1, imp::PixelType::i32fC1> ImageGpu32fC1;
typedef ImageGpu<imp::Pixel32fC2, imp::PixelType::i32fC2> ImageGpu32fC2;
typedef ImageGpu<imp::Pixel32fC3, imp::PixelType::i32fC3> ImageGpu32fC3;
typedef ImageGpu<imp::Pixel32fC4, imp::PixelType::i32fC4> ImageGpu32fC4;

template <typename Pixel, imp::PixelType pixel_type>
using ImageGpuPtr = typename std::shared_ptr<ImageGpu<Pixel,pixel_type>>;

} // namespace cu
} // namespace imp


#endif // IMP_CU_IMAGE_GPU_CUH
