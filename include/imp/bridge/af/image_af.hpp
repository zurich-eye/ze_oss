#pragma once

#include <arrayfire.h>
#include <af/internal.h>
#include <imp/core/image.hpp>

namespace ze {

class ImageAF32fC1 : public ImageBase
{
public:
  ImageAF32fC1() = delete;
  virtual ~ImageAF32fC1() = default;

  ImageAF32fC1(uint32_t width, uint32_t height);
  void fast()
  {
    af::features feat = af::fast(arr_, 20.0f, 9, true, 0.05);
    printf("Features found: %lu\n", feat.getNumFeatures());
  }

protected:
  af::array arr_;
};

} // ze namespace
