// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <memory>
#include <pangolin/image/image_io.h>
#include <imp/core/image_raw.hpp>

namespace ze
{

//------------------------------------------------------------------------------
//template<typename Pixel, imp::PixelType pixel_type>
void pangolinBridgeLoad(ze::ImageRaw8uC1::Ptr& out,
                        const std::string& filename, ze::PixelOrder pixel_order)
{
  // try to load an image with pangolin first
  pangolin::TypedImage im = pangolin::LoadImage(
        filename, pangolin::ImageFileType::ImageFileTypePng);

  //! @todo (MWE) FIX input output channel formatting, etc.
  out.reset(new ze::ImageRaw8uC1(reinterpret_cast<ze::Pixel8uC1*>(im.ptr),
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


} // namespace ze
