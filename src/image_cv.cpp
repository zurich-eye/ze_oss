#include <imp/bridge/opencv/image_cv.hpp>

#include <iostream>

#include <imp/core/exception.hpp>
#include <imp/bridge/opencv/cv_connector_pixel_types.hpp>


namespace ze {

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageCv<Pixel>::ImageCv(std::uint32_t width, std::uint32_t height)
  : Base(width, height)
  , mat_(height, width, ze::pixelTypeToCv(pixel_type<Pixel>::type))
{
  this->pitch_ = mat_.step;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageCv<Pixel>::ImageCv(const ze::Size2u& size)
  : Base(size)
  , mat_(size[1], size[0], ze::pixelTypeToCv(pixel_type<Pixel>::type))
{
  this->pitch_ = mat_.step;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageCv<Pixel>::ImageCv(const ImageCv<Pixel>& from)
  : Base(from)
  , mat_(from.cvMat())
{
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageCv<Pixel>::ImageCv(const Image<Pixel>& from)
  : Base(from)
  , mat_(from.height(), from.width(), ze::pixelTypeToCv(pixel_type<Pixel>::type))
{
  this->pitch_ = mat_.step;
  from.copyTo(*this);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageCv<Pixel>::ImageCv(cv::Mat mat, ze::PixelOrder pixel_order)
  : Base(mat.cols, mat.rows, pixel_order)
  , mat_(mat)
{
  this->pitch_ = mat_.step;
  if (this->pixelType() != ze::pixelTypeFromCv(mat_.type()))
  {
    throw ze::Exception("OpenCV pixel type does not match to the internally used one.",
                         __FILE__, __FUNCTION__, __LINE__);
  }

  if (this->pixelOrder() == ze::PixelOrder::undefined)
  {
    switch (this->pixelType())
    {
    case ze::PixelType::i8uC1:
    case ze::PixelType::i16uC1:
    case ze::PixelType::i32fC1:
    case ze::PixelType::i32sC1:
      this->pixel_order_ = ze::PixelOrder::gray;
      break;
    case ze::PixelType::i8uC3:
    case ze::PixelType::i16uC3:
    case ze::PixelType::i32fC3:
    case ze::PixelType::i32sC3:
      this->pixel_order_ = ze::PixelOrder::bgr;
      break;
    case ze::PixelType::i8uC4:
    case ze::PixelType::i16uC4:
    case ze::PixelType::i32fC4:
    case ze::PixelType::i32sC4:
      this->pixel_order_ = ze::PixelOrder::bgra;
      break;
    default:
      // if we have something else than 1,3 or 4-channel images, we do not set the
      // pixel order automatically.
      break;
    }
  }
}


////-----------------------------------------------------------------------------
//template<typename Pixel, imp::PixelType pixel_type>
//ImageCv<Pixel>
//::ImageCv(Pixel* data, std::uint32_t width, std::uint32_t height,
//           size_t pitch, bool use_ext_data_pointer)
//  : Base(width, height)
//{
//  if (data == nullptr)
//  {
//    throw imp::Exception("input data not valid", __FILE__, __FUNCTION__, __LINE__);
//  }

//  if(use_ext_data_pointer)
//  {
//    // This uses the external data pointer as internal data pointer.
//    auto dealloc_nop = [](Pixel* p) { ; };
//    data_ = std::unique_ptr<pixel_storage_t, Deallocator>(
//          data, Deallocator(dealloc_nop));
//    pitch_ = pitch;
//  }
//  else
//  {
//    data_.reset(Memory::alignedAlloc(this->width(), this->height(), &pitch_));
//    size_t stride = pitch / sizeof(pixel_storage_t);

//    if (this->bytes() == pitch*height)
//    {
//      std::copy(data, data+stride*height, data_.get());
//    }
//    else
//    {
//      for (std::uint32_t y=0; y<height; ++y)
//      {
//        for (std::uint32_t x=0; x<width; ++x)
//        {
//          data_.get()[y*this->stride()+x] = data[y*stride + x];
//        }
//      }
//    }
//  }
//}

//-----------------------------------------------------------------------------
template<typename Pixel>
cv::Mat& ImageCv<Pixel>::cvMat()
{
  return mat_;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
const cv::Mat& ImageCv<Pixel>::cvMat() const
{
  return mat_;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
Pixel* ImageCv<Pixel>::data(std::uint32_t ox, std::uint32_t oy)
{
  if (ox > this->width() || oy > this->height())
  {
    throw ze::Exception("Request starting offset is outside of the image.", __FILE__, __FUNCTION__, __LINE__);
  }
  Pixel* buffer = (Pixel*)mat_.data;
  return &buffer[oy*this->stride() + ox];
}

//-----------------------------------------------------------------------------
template<typename Pixel>
const Pixel* ImageCv<Pixel>::data(std::uint32_t ox, std::uint32_t oy) const
{
  if (ox > this->width() || oy > this->height())
  {
    throw ze::Exception("Request starting offset is outside of the image.", __FILE__, __FUNCTION__, __LINE__);
  }

  Pixel* buffer = (Pixel*)mat_.data;
  return reinterpret_cast<const Pixel*>(&buffer[oy*this->stride() + ox]);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void ImageCv<Pixel>::setValue(const Pixel& value)
{
  mat_ = cv::Scalar::all(value);
}


//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class ImageCv<ze::Pixel8uC1>;
template class ImageCv<ze::Pixel8uC2>;
template class ImageCv<ze::Pixel8uC3>;
template class ImageCv<ze::Pixel8uC4>;

template class ImageCv<ze::Pixel16uC1>;
template class ImageCv<ze::Pixel16uC2>;
template class ImageCv<ze::Pixel16uC3>;
template class ImageCv<ze::Pixel16uC4>;

template class ImageCv<ze::Pixel32sC1>;
template class ImageCv<ze::Pixel32sC2>;
template class ImageCv<ze::Pixel32sC3>;
template class ImageCv<ze::Pixel32sC4>;

template class ImageCv<ze::Pixel32fC1>;
template class ImageCv<ze::Pixel32fC2>;
template class ImageCv<ze::Pixel32fC3>;
template class ImageCv<ze::Pixel32fC4>;


} // namespace ze
