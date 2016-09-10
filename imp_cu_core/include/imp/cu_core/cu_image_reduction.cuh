#pragma once

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_linearmemory.cuh>

namespace ze {
namespace cu {

template<typename Pixel>
class ImageReducer
{
public:
  ImageReducer();
  ~ImageReducer();

  // Sum image by reduction
  Pixel sum(const ImageGpu<Pixel>& in_img);

  // Count elements equal to 'value'
  size_t countEqual(
      const ImageGpu32sC1& in_img,
      int32_t value);

private:
  Fragmentation<> fragm_{dim3(4, 4, 1), dim3(16, 16, 1)};
  unsigned int sh_mem_size_;
  cu::LinearMemory<Pixel> dev_final_{1};
  ImageGpu<Pixel> partial_;
};

} // cu namespace
} // ze namespace
