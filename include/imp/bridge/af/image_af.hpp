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

  const af::array& afArray() const;

protected:
  af::array arr_;

private:
  static af_dtype pixelTypeToAF(ze::PixelType type);
};

typedef ImageAF<ze::Pixel8uC1> ImageAF8uC1;
typedef ImageAF<ze::Pixel32sC1> ImageAF32sC1;
typedef ImageAF<ze::Pixel32fC1> ImageAF32fC1;

} // ze namespace
