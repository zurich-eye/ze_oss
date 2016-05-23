#include <imp/bridge/af/image_af.hpp>

namespace ze {

template<typename Pixel>
af_dtype ImageAF<Pixel>::pixelTypeToAF(ze::PixelType type)
{
  switch (type)
  {
  case PixelType::i8uC1: return u8;
  case PixelType::i32sC1: return s32;
  case PixelType::i32fC1: return f32;
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
  CHECK_EQ(ox, 0);
  CHECK_EQ(oy, 0);
  switch(pixel_type<Pixel>::type)
  {
  case PixelType::i8uC1:
    return reinterpret_cast<Pixel*>(arr_.device<uint8_t>());
  case PixelType::i32sC1:
    return reinterpret_cast<Pixel*>(arr_.device<int>());;
  case PixelType::i32fC1:
    return reinterpret_cast<Pixel*>(arr_.device<float>());;
  default: IMP_THROW_EXCEPTION("pixel type not supported");
  }
}

template<typename Pixel>
const Pixel* ImageAF<Pixel>::data(uint32_t ox, uint32_t oy) const
{
  CHECK_EQ(ox, 0);
  CHECK_EQ(oy, 0);
  switch(pixel_type<Pixel>::type)
  {
  case PixelType::i8uC1:
    return reinterpret_cast<const Pixel*>(arr_.device<uint8_t>());
  case PixelType::i32sC1:
    return reinterpret_cast<const Pixel*>(arr_.device<int>());;
  case PixelType::i32fC1:
    return reinterpret_cast<const Pixel*>(arr_.device<float>());;
  default: IMP_THROW_EXCEPTION("pixel type not supported");
  }
}

template<typename Pixel>
const af::array& ImageAF<Pixel>::afArray() const
{
  return arr_;
}

template class ImageAF<ze::Pixel8uC1>;
template class ImageAF<ze::Pixel32sC1>;
template class ImageAF<ze::Pixel32fC1>;

} // ze namespace
