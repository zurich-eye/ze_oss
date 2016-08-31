#include <imp/bridge/af/image_af.hpp>
#include <imp/core/linearmemory.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace ze {

template<typename Pixel>
struct ImpAfConversionBuffer
{
  ze::LinearMemory<Pixel> h_buff;

  ImpAfConversionBuffer(const Image<Pixel>& from)
    : h_buff(from.numel())
  {
    for (size_t y = 0; y < from.height(); ++y)
    {
      for (size_t x = 0; x < from.width(); ++x)
      {
        h_buff(x*from.height()+y) = from.pixel(x, y);  //! AF array is column-major
      }
    }
  }

  auto cuData() -> decltype(ze::cu::toCudaVectorType(this->h_buff.data()))
  {
    return ze::cu::toCudaVectorType(this->h_buff.data());
  }

  auto cuData() const -> decltype(ze::cu::toConstCudaVectorType(this->h_buff.data()))
  {
    return ze::cu::toConstCudaVectorType(this->h_buff.data());
  }
};

template<typename Pixel>
ImageAF<Pixel>::ImageAF(const Image<Pixel>& from)
  : Image<Pixel>(from)
{
  ImpAfConversionBuffer<Pixel> buffer(from);
  arr_ = af::array(from.height(), from.width(), buffer.cuData());
}

template<typename Pixel>
ImageAF<Pixel>::ImageAF(const af::array& from)
  : Image<Pixel>(ze::Size2u(from.dims()[1], from.dims()[0])),
    arr_(from)
{ }

template<typename Pixel>
Pixel* ImageAF<Pixel>::data(uint32_t ox, uint32_t oy)
{
  DEBUG_CHECK_EQ(ox, 0u);
  DEBUG_CHECK_EQ(oy, 0u);
  switch(pixel_type<Pixel>::type)
  {
  case PixelType::i8uC1:
    return reinterpret_cast<Pixel*>(arr_.device<unsigned char>());
  case PixelType::i32sC1:
    return reinterpret_cast<Pixel*>(arr_.device<int>());
  case PixelType::i32fC1:
    return reinterpret_cast<Pixel*>(arr_.device<float>());
  default:
    LOG(FATAL) << "pixel type not supported";
    break;
  }
}

template<typename Pixel>
const Pixel* ImageAF<Pixel>::data(uint32_t ox, uint32_t oy) const
{
  DEBUG_CHECK_EQ(ox, 0u);
  DEBUG_CHECK_EQ(oy, 0u);
  switch(pixel_type<Pixel>::type)
  {
  case PixelType::i8uC1:
    return reinterpret_cast<const Pixel*>(arr_.device<unsigned char>());
  case PixelType::i32sC1:
    return reinterpret_cast<const Pixel*>(arr_.device<int>());
  case PixelType::i32fC1:
    return reinterpret_cast<const Pixel*>(arr_.device<float>());
  default:
    LOG(FATAL) << "pixel type not supported";
    break;
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
