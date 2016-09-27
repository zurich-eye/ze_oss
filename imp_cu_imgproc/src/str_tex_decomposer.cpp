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
#include <imp/cu_imgproc/str_tex_decomposer.hpp>

#include <imp/cu_core/cu_math.cuh>
#include <imp/cu_imgproc/cu_rof_denoising.cuh>
#include <ze/common/logging.hpp>

namespace ze {
namespace cu {
//-----------------------------------------------------------------------------
template<typename Pixel>
StrTexDecomposer<Pixel>::StrTexDecomposer()
  : denoiser_(new RofDenoising<Pixel>())
{

}

//-----------------------------------------------------------------------------
template<typename Pixel>
void StrTexDecomposer<Pixel>::solve(
    const ze::cu::ImageGpuPtr<Pixel>& tex_image,
    const ze::cu::ImageGpuPtr<Pixel>& src,
    const ze::cu::ImageGpuPtr<Pixel>& structure_image)
{
  CHECK(src);
  CHECK(tex_image);

  CHECK_EQ(src->size(), tex_image->size());
  if (structure_image)
  {
    CHECK_EQ(src->size(), structure_image->size());
  }

  src_ = src;
  tex_ = tex_image;
  // in-place denoising if no structure image desired for an output
  str_ = structure_image ? structure_image : tex_image;

  denoiser_->params().lambda = 1.0f;
  denoiser_->params().max_iter = 100;
  denoiser_->denoise(str_, src_);
  ze::cu::weightedSum(*tex_, *src_, weight_, *str_, 1.f-2.f*weight_);
}

//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class StrTexDecomposer<ze::Pixel8uC1>;
template class StrTexDecomposer<ze::Pixel32fC1>;


} // namespace cu
} // namespace ze
