#include <imp/bridge/af/image_af.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace ze {

template<typename Pixel>
class ImpAfConversionBuffer
{
private:
  std::unique_ptr<Pixel[]> data_;
public:
  ImpAfConversionBuffer(const Image<Pixel>& from)
  {
    data_.reset(new Pixel[from.width()*from.height()]);
    for(size_t r=0; r < from.height(); ++r)
    {
      for(size_t c=0; c < from.width(); ++c)
      {
        data_.get()[c*from.height()+r] = from.pixel(c, r);
      }
    }
  }
  auto cuData() -> decltype(ze::cu::toCudaVectorType(this->data_.get()))
  {
    return ze::cu::toCudaVectorType(this->data_.get());
  }
  auto cuData() const -> decltype(ze::cu::toConstCudaVectorType(this->data_.get()))
  {
    return ze::cu::toConstCudaVectorType(this->data_.get());
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
  CHECK_EQ(ox, 0);
  CHECK_EQ(oy, 0);
  switch(pixel_type<Pixel>::type)
  {
  case PixelType::i8uC1:
    return reinterpret_cast<Pixel*>(arr_.device<unsigned char>());
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
    return reinterpret_cast<const Pixel*>(arr_.device<unsigned char>());
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
