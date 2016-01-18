#include <imp/core/image_raw.hpp>

#include <iostream>

#include <imp/core/exception.hpp>


namespace ze {

//-----------------------------------------------------------------------------
template<typename Pixel, ze::PixelType pixel_type>
ImageRaw<Pixel, pixel_type>::ImageRaw(std::uint32_t width, std::uint32_t height)
  : Base(width, height)
{
  data_.reset(Memory::alignedAlloc(width, height, &pitch_));
}

//-----------------------------------------------------------------------------
template<typename Pixel, ze::PixelType pixel_type>
ImageRaw<Pixel, pixel_type>::ImageRaw(const ze::Size2u& size)
  : Base(size)
{
  data_.reset(Memory::alignedAlloc(size, &pitch_));
}

//-----------------------------------------------------------------------------
template<typename Pixel, ze::PixelType pixel_type>
ImageRaw<Pixel, pixel_type>::ImageRaw(const ImageRaw& from)
  : Base(from)
{
  data_.reset(Memory::alignedAlloc(this->width(), this->height(), &pitch_));
  from.copyTo(*this);
}

//-----------------------------------------------------------------------------
template<typename Pixel, ze::PixelType pixel_type>
ImageRaw<Pixel, pixel_type>::ImageRaw(const Image<Pixel, pixel_type>& from)
  : Base(from)
{
  data_.reset(Memory::alignedAlloc(this->width(), this->height(), &pitch_));
  from.copyTo(*this);
}

//-----------------------------------------------------------------------------
template<typename Pixel, ze::PixelType pixel_type>
ImageRaw<Pixel, pixel_type>
::ImageRaw(Pixel* data, std::uint32_t width, std::uint32_t height,
           size_t pitch, bool use_ext_data_pointer)
  : Base(width, height)
{
  if (data == nullptr)
  {
    throw ze::Exception("input data not valid", __FILE__, __FUNCTION__, __LINE__);
  }

  if(use_ext_data_pointer)
  {
    // This uses the external data pointer as internal data pointer.
    auto dealloc_nop = [](Pixel*) { ; };
    data_ = std::unique_ptr<Pixel, Deallocator>(data, Deallocator(dealloc_nop));
    pitch_ = pitch;
  }
  else
  {
    data_.reset(Memory::alignedAlloc(this->width(), this->height(), &pitch_));
    size_t stride = pitch / sizeof(Pixel);

    if (this->bytes() == pitch*height)
    {
      std::copy(data, data+stride*height, data_.get());
    }
    else
    {
      for (std::uint32_t y=0; y<height; ++y)
      {
        for (std::uint32_t x=0; x<width; ++x)
        {
          data_.get()[y*this->stride()+x] = data[y*stride + x];
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel, ze::PixelType pixel_type>
ImageRaw<Pixel, pixel_type>::ImageRaw(Pixel* data,
                                      std::uint32_t width, std::uint32_t height,
                                      size_t pitch,
                                      const std::shared_ptr<void const>& tracked)
  : Base(width, height)
{
  if (data == nullptr || tracked == nullptr)
  {
    throw ze::Exception("input data not valid", __FILE__, __FUNCTION__, __LINE__);
  }

  auto dealloc_nop = [](Pixel*) { ; };
  data_ = std::unique_ptr<Pixel, Deallocator>(data, Deallocator(dealloc_nop));
  pitch_ = pitch;
  tracked_ = tracked;
}

//-----------------------------------------------------------------------------
template<typename Pixel, ze::PixelType pixel_type>
Pixel* ImageRaw<Pixel, pixel_type>::data(
    std::uint32_t ox, std::uint32_t oy)
{
  if (ox > this->width() || oy > this->height())
  {
    throw ze::Exception("Request starting offset is outside of the image.", __FILE__, __FUNCTION__, __LINE__);
  }

  return &data_.get()[oy*this->stride() + ox];
}

//-----------------------------------------------------------------------------
template<typename Pixel, ze::PixelType pixel_type>
const Pixel* ImageRaw<Pixel, pixel_type>::data(
    std::uint32_t ox, std::uint32_t oy) const
{
  if (ox > this->width() || oy > this->height())
  {
    throw ze::Exception("Request starting offset is outside of the image.", __FILE__, __FUNCTION__, __LINE__);
  }

  return reinterpret_cast<const Pixel*>(&data_.get()[oy*this->stride() + ox]);
}

//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class ImageRaw<ze::Pixel8uC1, ze::PixelType::i8uC1>;
template class ImageRaw<ze::Pixel8uC2, ze::PixelType::i8uC2>;
template class ImageRaw<ze::Pixel8uC3, ze::PixelType::i8uC3>;
template class ImageRaw<ze::Pixel8uC4, ze::PixelType::i8uC4>;

template class ImageRaw<ze::Pixel16sC1, ze::PixelType::i16sC1>;
template class ImageRaw<ze::Pixel16sC2, ze::PixelType::i16sC2>;
template class ImageRaw<ze::Pixel16sC3, ze::PixelType::i16sC3>;
template class ImageRaw<ze::Pixel16sC4, ze::PixelType::i16sC4>;

template class ImageRaw<ze::Pixel16uC1, ze::PixelType::i16uC1>;
template class ImageRaw<ze::Pixel16uC2, ze::PixelType::i16uC2>;
template class ImageRaw<ze::Pixel16uC3, ze::PixelType::i16uC3>;
template class ImageRaw<ze::Pixel16uC4, ze::PixelType::i16uC4>;

template class ImageRaw<ze::Pixel32sC1, ze::PixelType::i32sC1>;
template class ImageRaw<ze::Pixel32sC2, ze::PixelType::i32sC2>;
template class ImageRaw<ze::Pixel32sC3, ze::PixelType::i32sC3>;
template class ImageRaw<ze::Pixel32sC4, ze::PixelType::i32sC4>;

template class ImageRaw<ze::Pixel32fC1, ze::PixelType::i32fC1>;
template class ImageRaw<ze::Pixel32fC2, ze::PixelType::i32fC2>;
template class ImageRaw<ze::Pixel32fC3, ze::PixelType::i32fC3>;
template class ImageRaw<ze::Pixel32fC4, ze::PixelType::i32fC4>;

} // namespace imp
