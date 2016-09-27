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

#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>


namespace ze {
namespace cu {

//------------------------------------------------------------------------------
template<typename Pixel>
void cvBridgeLoad(ze::cu::ImageGpuPtr<Pixel>& out, const std::string& filename,
                  ze::PixelOrder pixel_order)
{
  ImageCvPtr<Pixel> cv_img;
  ze::cvBridgeLoad<Pixel>(cv_img, filename, pixel_order);
  out = std::make_shared<ze::cu::ImageGpu<Pixel>>(*cv_img);
}

//------------------------------------------------------------------------------
template<typename Pixel>
void cvBridgeShow(const std::string& winname,
                  const ze::cu::ImageGpu<Pixel>& img, bool normalize=false)
{
  const ImageCv<Pixel> cv_img(img);
  ze::cvBridgeShow(winname, cv_img, normalize);
}

//------------------------------------------------------------------------------
template<typename Pixel, typename T>
void cvBridgeShow(const std::string& winname,
                  const ze::cu::ImageGpu<Pixel>& img,
                  const T& min, const T& max)
{
  const ImageCv<Pixel> cv_img(img);
  ze::cvBridgeShow(winname, cv_img, min, max);
}

} // namespace cu
} // namespace ze
