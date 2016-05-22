#include <imp/bridge/af/image_af.hpp>

namespace ze {

af_dtype pixelTypeToAF(ze::PixelType type)
{
  switch (type)
  {
  case ze::PixelType::i8uC1: return u8;
  case ze::PixelType::i32sC1: return s32;
  case ze::PixelType::i32fC1: return f32;
  default: IMP_THROW_EXCEPTION("pixel type not supported");
  }
}

template<typename Pixel>
ImageAF<Pixel>::ImageAF(const Image<Pixel>& from)
  : Image<Pixel>(from)
{
  arr_ = af::createStridedArray(
        from.data(), 0, af::dim4(from.width(),from.height(), 1, 1),
        af::dim4(1, from.stride(), 1, 1),
        pixelTypeToAF(pixel_type<Pixel>::type), afHost);
  arr_ = arr_.T();
}

template<typename Pixel>
Pixel* ImageAF<Pixel>::data(uint32_t ox, uint32_t oy)
{
  CHECK_LT(ox, this->width());
  CHECK_LT(oy, this->height());

  return nullptr;
  //Pixel* buffer = (Pixel*)mat_.data;
  //return &buffer[oy*this->stride() + ox];
}

//-----------------------------------------------------------------------------
template<typename Pixel>
const Pixel* ImageAF<Pixel>::data(uint32_t ox, uint32_t oy) const
{
  CHECK_LT(ox, this->width());
  CHECK_LT(oy, this->height());

  return nullptr;
  //Pixel* buffer = (Pixel*)mat_.data;
  //return reinterpret_cast<const Pixel*>(&buffer[oy*this->stride() + ox]);
}

template class ImageAF<ze::Pixel8uC1>;
template class ImageAF<ze::Pixel32sC1>;
template class ImageAF<ze::Pixel32fC1>;

} // ze namespace
