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

#include <imp/cu_imgproc/cu_variational_denoising.cuh>

#include <memory>
#include <cuda_runtime_api.h>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/common/macros.hpp>

namespace ze {
namespace cu {

/** Decomposes the input image into its structure (cartoon) and texture part. */
template<typename Pixel>
class StrTexDecomposer
{
public:
  ZE_POINTER_TYPEDEFS(StrTexDecomposer);
  using Base = VariationalDenoising;
  using ImageGpu = ze::cu::ImageGpu<Pixel>;

public:
  StrTexDecomposer();
  ~StrTexDecomposer() = default;

  void solve(const ze::cu::ImageGpuPtr<Pixel>& tex_image,
             const ze::cu::ImageGpuPtr<Pixel>& src,
             const ze::cu::ImageGpuPtr<Pixel>& structure_image = NULL);

private:
  ImageGpuPtr<Pixel> src_;
  ImageGpuPtr<Pixel> denoised_;
  ImageGpuPtr<Pixel> str_;
  ImageGpuPtr<Pixel> tex_;
  ze::cu::VariationalDenoising::Ptr denoiser_;
  float weight_ = 0.8; //!< weighting of denoised towards original image during decomposition
};

//-----------------------------------------------------------------------------
// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef StrTexDecomposer<ze::Pixel8uC1> StrTexDecomposer8uC1;
typedef StrTexDecomposer<ze::Pixel32fC1> StrTexDecomposer32fC1;

template <typename Pixel>
using StrTexDecomposerPtr = typename std::shared_ptr<StrTexDecomposer<Pixel>>;

} // namespace cu
} // namespace ze
