#pragma once

#include <imp/cu_core/cu_utils.hpp>

namespace ze {
namespace cu {

template<typename T>
class ImageReducer
{
public:
  ImageReducer();
  ~ImageReducer();

  // Sum image by reduction
  T sum(const T *in_img_data,
        size_t in_img_stride,
        size_t in_img_width,
        size_t in_img_height);

  // Count elements equal to 'value'
  size_t countEqual(const int *in_img_data,
                    size_t in_img_stride,
                    size_t in_img_width,
                    size_t in_img_height,
                    int value);

private:
  ze::cu::Fragmentation<> fragm_;
  unsigned int sh_mem_size_;
  T* dev_final_;
  T* dev_partial_;
  size_t dev_partial_pitch_;
  size_t dev_partial_stride_;
  bool is_dev_part_alloc_;
  bool is_dev_fin_alloc_;
};

} // cu namespace
} // ze namespace
