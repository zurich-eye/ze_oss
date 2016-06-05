#pragma once

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>

namespace ze {
namespace cu {

template<typename Pixel>
class ImageReducer
{
public:
  ImageReducer();
  ~ImageReducer();

  // Sum image by reduction
  Pixel sum(const ze::cu::ImageGpu<Pixel>& in_img);

  // Count elements equal to 'value'
  size_t countEqual(
      const ze::cu::ImageGpu32sC1& in_img,
      int32_t value);

private:
  ze::cu::Fragmentation<> fragm_;
  unsigned int sh_mem_size_;
  Pixel* dev_final_;
  ze::cu::ImageGpu<Pixel> partial_;
};

} // cu namespace
} // ze namespace
