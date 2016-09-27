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
