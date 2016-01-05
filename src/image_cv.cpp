#include <imp/bridge/opencv/image_cv.hpp>

#include <iostream>

#include <imp/core/exception.hpp>
#include <imp/bridge/opencv/cv_connector_pixel_types.hpp>


namespace imp {

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
ImageCv<Pixel, pixel_type>::ImageCv(std::uint32_t width, std::uint32_t height)
  : Base(width, height)
  , mat_(height, width, imp::pixelTypeToCv(pixel_type))
{
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
ImageCv<Pixel, pixel_type>::ImageCv(const imp::Size2u& size)
  : Base(size)
  , mat_(size[1], size[0], imp::pixelTypeToCv(pixel_type))
{
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
ImageCv<Pixel, pixel_type>::ImageCv(const ImageCv<Pixel, pixel_type>& from)
  : Base(from)
  , mat_(from.cvMat())
{
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
ImageCv<Pixel, pixel_type>::ImageCv(const Image<Pixel, pixel_type>& from)
  : Base(from)
  , mat_(from.height(), from.width(), imp::pixelTypeToCv(pixel_type))
{
  from.copyTo(*this);
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
ImageCv<Pixel, pixel_type>::ImageCv(cv::Mat mat, imp::PixelOrder pixel_order)
  : Base(mat.cols, mat.rows, pixel_order)
  , mat_(mat)
{
  if (this->pixelType() != imp::pixelTypeFromCv(mat_.type()))
  {
    throw imp::Exception("OpenCV pixel type does not match to the internally used one.",
                         __FILE__, __FUNCTION__, __LINE__);
  }

  if (this->pixelOrder() == imp::PixelOrder::undefined)
  {
    switch (this->pixelType())
    {
    case imp::PixelType::i8uC1:
    case imp::PixelType::i16uC1:
    case imp::PixelType::i32fC1:
    case imp::PixelType::i32sC1:
      this->pixel_order_ = imp::PixelOrder::gray;
      break;
    case imp::PixelType::i8uC3:
    case imp::PixelType::i16uC3:
    case imp::PixelType::i32fC3:
    case imp::PixelType::i32sC3:
      this->pixel_order_ = imp::PixelOrder::bgr;
      break;
    case imp::PixelType::i8uC4:
    case imp::PixelType::i16uC4:
    case imp::PixelType::i32fC4:
    case imp::PixelType::i32sC4:
      this->pixel_order_ = imp::PixelOrder::bgra;
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
//ImageCv<Pixel, pixel_type>
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
template<typename Pixel, imp::PixelType pixel_type>
cv::Mat& ImageCv<Pixel, pixel_type>::cvMat()
{
  return mat_;
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
const cv::Mat& ImageCv<Pixel, pixel_type>::cvMat() const
{
  return mat_;
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
Pixel* ImageCv<Pixel, pixel_type>::data(
    std::uint32_t ox, std::uint32_t oy)
{
  if (ox > this->width() || oy > this->height())
  {
    throw imp::Exception("Request starting offset is outside of the image.", __FILE__, __FUNCTION__, __LINE__);
  }
  Pixel* buffer = (Pixel*)mat_.data;
  return &buffer[oy*this->stride() + ox];
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
const Pixel* ImageCv<Pixel, pixel_type>::data(
    std::uint32_t ox, std::uint32_t oy) const
{
  if (ox > this->width() || oy > this->height())
  {
    throw imp::Exception("Request starting offset is outside of the image.", __FILE__, __FUNCTION__, __LINE__);
  }

  Pixel* buffer = (Pixel*)mat_.data;
  return reinterpret_cast<const Pixel*>(&buffer[oy*this->stride() + ox]);
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void ImageCv<Pixel,pixel_type>::setValue(const Pixel& value)
{
  mat_ = cv::Scalar::all(value);
}


//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class ImageCv<imp::Pixel8uC1, imp::PixelType::i8uC1>;
template class ImageCv<imp::Pixel8uC2, imp::PixelType::i8uC2>;
template class ImageCv<imp::Pixel8uC3, imp::PixelType::i8uC3>;
template class ImageCv<imp::Pixel8uC4, imp::PixelType::i8uC4>;

template class ImageCv<imp::Pixel16uC1, imp::PixelType::i16uC1>;
template class ImageCv<imp::Pixel16uC2, imp::PixelType::i16uC2>;
template class ImageCv<imp::Pixel16uC3, imp::PixelType::i16uC3>;
template class ImageCv<imp::Pixel16uC4, imp::PixelType::i16uC4>;

template class ImageCv<imp::Pixel32sC1, imp::PixelType::i32sC1>;
template class ImageCv<imp::Pixel32sC2, imp::PixelType::i32sC2>;
template class ImageCv<imp::Pixel32sC3, imp::PixelType::i32sC3>;
template class ImageCv<imp::Pixel32sC4, imp::PixelType::i32sC4>;

template class ImageCv<imp::Pixel32fC1, imp::PixelType::i32fC1>;
template class ImageCv<imp::Pixel32fC2, imp::PixelType::i32fC2>;
template class ImageCv<imp::Pixel32fC3, imp::PixelType::i32fC3>;
template class ImageCv<imp::Pixel32fC4, imp::PixelType::i32fC4>;


} // namespace imp
