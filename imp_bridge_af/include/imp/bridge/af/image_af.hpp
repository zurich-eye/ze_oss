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

#include <arrayfire.h>
#include <af/internal.h>
#include <imp/core/image.hpp>

namespace ze {

template<typename Pixel>
class ImageAF : public ze::Image<Pixel>
{
public:
  using Ptr = typename std::shared_ptr<ImageAF<Pixel>>;
  using ConstPtrRef = const Ptr&;
  using ConstPtr = typename std::shared_ptr<ImageAF<Pixel> const>;

public:
  ImageAF() = delete;
  virtual ~ImageAF() = default;

  ImageAF(const Image<Pixel>& from);
  ImageAF(const af::array& from);

  virtual Pixel* data(uint32_t ox = 0, uint32_t oy = 0) override;
  virtual const Pixel* data(uint32_t ox = 0, uint32_t oy = 0) const override;

  const af::array& afArray() const;

protected:
  af::array arr_;

private:
  static af::dtype pixelTypeToAF(ze::PixelType type);
};

typedef ImageAF<ze::Pixel8uC1> ImageAF8uC1;
typedef ImageAF<ze::Pixel32sC1> ImageAF32sC1;
typedef ImageAF<ze::Pixel32fC1> ImageAF32fC1;

} // ze namespace
