#pragma once

#include <arrayfire.h>
#include <af/internal.h>
#include <imp/core/image.hpp>

namespace ze {

template<typename Pixel>
class ImageAF : public ze::Image<Pixel>
{
public:
  using Ptr = typename std::shared_ptr<ImageAF<Pixel>>;
  using ConstPtrRef = const Ptr&;
  using ConstPtr = typename std::shared_ptr<ImageAF<Pixel> const>;

public:
  ImageAF() = delete;
  virtual ~ImageAF() = default;

  ImageAF(const Image<Pixel>& from);

  virtual Pixel* data(uint32_t ox = 0, uint32_t oy = 0) override;
  virtual const Pixel* data(uint32_t ox = 0, uint32_t oy = 0) const override;

  void fast()
  {
    af::features feat = af::fast(arr_*255.f, 20.0f, 9, true, 0.05);
    printf("Features found: %lu\n", feat.getNumFeatures());
  }

  void display()
  {
    af::Window wnd("AF array");

    // Previews color image with green crosshairs
    while(!wnd.close())
        wnd.image(arr_);
  }

protected:
  af::array arr_;
};

typedef ImageAF<ze::Pixel8uC1> ImageAF8uC1;
typedef ImageAF<ze::Pixel32sC1> ImageAF32sC1;
typedef ImageAF<ze::Pixel32fC1> ImageAF32fC1;

} // ze namespace
