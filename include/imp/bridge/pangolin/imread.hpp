#pragma once

#include <memory>
#include <pangolin/image/image_io.h>
#include <imp/core/image_raw.hpp>

namespace imp
{

//------------------------------------------------------------------------------
//template<typename Pixel, imp::PixelType pixel_type>
void pangolinBridgeLoad(imp::ImageRaw8uC1::Ptr& out,
                        const std::string& filename, imp::PixelOrder pixel_order)
{
  // try to load an image with pangolin first
  pangolin::TypedImage im = pangolin::LoadImage(
        filename, pangolin::ImageFileType::ImageFileTypePng);

  //! @todo (MWE) FIX input output channel formatting, etc.
  out.reset(new imp::ImageRaw8uC1(reinterpret_cast<imp::Pixel8uC1*>(im.ptr),
                                  im.w, im.h, im.pitch));
//  switch (im.fmt.channels)
//  {
//  case 1:
//    out = std::make_shared<imp::ImageRaw<Pixel, pixel_type>>(im.w, im.h);
//    break;
//  case 3:
//    out = std::make_shared<imp::ImageRaw<Pixel, pixel_type>>(im.w, im.h);
//    break;
//  case 4:
//    out = std::make_shared<imp::ImageRaw<Pixel, pixel_type>>(im.w, im.h);
//    break;
//  default:
//    throw imp::Exception("Conversion for reading given pixel_type not supported yet.", __FILE__, __FUNCTION__, __LINE__);

//  }
}


} // namespace imp
