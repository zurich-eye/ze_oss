#pragma once

#include <memory>
#include <cstdint>

#include <glog/logging.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <imp/core/exception.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>

using imgenc = sensor_msgs::image_encodings;

namespace ze {

//------------------------------------------------------------------------------
void getPixelTypeFromRosImageEncoding(
    ze::PixelType& pixel_type,
    ze::PixelOrder& pixel_order,
    const std::string& encoding)
{
  //! @todo (MWE) we do not support bayer or YUV images yet.
  if (encoding == enc::BGR8)
  {
    pixel_type = ze::PixelType::i8uC3;
    pixel_order = ze::PixelOrder::bgr;
  }
  else if (encoding == enc::MONO8)
  {
    pixel_type = ze::PixelType::i8uC1;
    pixel_order = ze::PixelOrder::gray;
  }
  else if (encoding == enc::RGB8)
  {
    pixel_type = ze::PixelType::i8uC3;
    pixel_order = ze::PixelOrder::rgb;
  }
  else if (encoding == enc::MONO16)
  {
    pixel_type = ze::PixelType::i16C1;
    pixel_order = ze::PixelOrder::gray;
  }
  else if (encoding == enc::BGR16)
  {
    pixel_type = ze::PixelType::i16uC3;
    pixel_order = ze::PixelOrder::bgr;
  }
  else if (encoding == enc::RGB16)
  {
    pixel_type = ze::PixelType::i16uC3;
    pixel_order = ze::PixelOrder::rgb;
  }
  else if (encoding == enc::BGRA8)
  {
    pixel_type = ze::PixelType::i8uC4;
    pixel_order = ze::PixelOrder::bgra;
  }
  else if (encoding == enc::RGBA8)
  {
    pixel_type = ze::PixelType::i8uC4;
    pixel_order = ze::PixelOrder::rgba;
  }
  else if (encoding == enc::BGRA16)
  {
    pixel_type = ze::PixelType::i16uC4;
    pixel_order = ze::PixelOrder::bgra;
  }
  else if (encoding == enc::RGBA16)
  {
    pixel_type = ze::PixelType::i16uC4;
    pixel_order = ze::PixelOrder::rgba;
  }
  else
  {
    IMP_THROW_EXCEPTION("Unsupported image encoding " + encoding + ".");
  }
}

////------------------------------------------------------------------------------
//template<typename Pixel, imp::PixelType pixel_type>
//void rosBridge(ImageRawPtr<Pixel,pixel_type>& out,
//               const sensor_msgs::ImageConstPtr& img_msg,
//               imp::PixelOrder pixel_order)
//{
//  // TODO
//}


//------------------------------------------------------------------------------
template<typename Pixel, ze::PixelType pixel_type>
ze::cu::ImageGpu<Pixel,pixel_type>::Ptr toImageGpu(
    const sensor_msgs::Image& src/*,
    imp::PixelOrder pixel_order*/)
{
  ze::PixelType src_pixel_type;
  ze::PixelOrder src_pixel_order;
  ze::getPixelTypeFromRosImageEncoding(src_pixel_type, src_pixel_order, src.encoding);


  int bit_depth = imgenc::bitDepth(src.encoding);
  int num_channels = imgenc::numChannels(src.encoding);
  std::uint32_t width = src.width;
  std::uint32_t height = src.height;
  std::uint32_t pitch = src.step;

  // sanity check
  CHECK_LT(pitch, width * num_channels * bit_depth/8) << "Input image seem to wrongly formatted";

  switch (src_pixel_type)
  {
  case ze::PixelType::i8uC1:
  {
    ze::ImageRaw8uC1 src_wrapped(
          const_cast<ze::Pixel8uC1*>(&src.data[0]),
        width, height, pitch, true);
    ze::cu::ImageGpu8uC1::Ptr dst =
        std::make_shared<ze::cu::ImageGpu (width, height);
    dst->copyFrom(src_wrapped);
    return dst;
  }
  break;
//  case imp::PixelType::i8uC2:
//  {
//  }
//  break;
//  case imp::PixelType::i8uC3:
//  {
//  }
//  break;
//  case imp::PixelType::i8uC4:
//  {
//  }
//  break;
//  case imp::PixelType::i16uC1:
//  {
//  }
//  break;
//  case imp::PixelType::i16uC2:
//  {
//  }
//  break;
//  case imp::PixelType::i16uC3:
//  {
//  }
//  break;
//  case imp::PixelType::i16uC4:
//  {
//  }
//  break;
//  case imp::PixelType::i32uC1:
//  {
//  }
//  break;
//  case imp::PixelType::i32uC2:
//  {
//  }
//  break;
//  case imp::PixelType::i32uC3:
//  {
//  }
//  break;
//  case imp::PixelType::i32uC4:
//  {
//  }
//  break;
//  case imp::PixelType::i32sC1:
//  {
//  }
//  break;
//  case imp::PixelType::i32sC2:
//  {
//  }
//  break;
//  case imp::PixelType::i32sC3:
//  {
//  }
//  break;
//  case imp::PixelType::i32sC4:
//  {
//  }
//  break;
//  case imp::PixelType::i32fC1:
//  {
//  }
//  break;
//  case imp::PixelType::i32fC2:
//  {
//  }
//  break;
//  case imp::PixelType::i32fC3:
//  {
//  }
//  break;
//  case imp::PixelType::i32fC4:
//  {

//  }
//  break;
  default:
    IMP_THROW_EXCEPTION("Unsupported pixel type" + encoding + ".");
  } // switch(...)



} // namespace imp

