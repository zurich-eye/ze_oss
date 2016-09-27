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
#include <imp/cu_correspondence/occlusion.cuh>

#include <cuda_runtime_api.h>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_texture.cuh>

namespace ze {
namespace cu {

//------------------------------------------------------------------------------
__global__ void k_occlusionCandidatesUniqunessMapping(
    float* occ, size_t stride, uint32_t width, uint32_t height,
    Texture2D disp_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x /*+ roi_x*/;
  const int y = blockIdx.y*blockDim.y + threadIdx.y /*+ roi_y*/;

  const int wx = x + static_cast<int>(tex2DFetch<float>(disp_tex, x, y) + 0.5f);

  if (wx>0 && wx<width && y<height)
  {
    atomicAdd(&occ[y*stride+wx], 1.f);
  }
}

template<typename Pixel>
__host__ __device__ Pixel clamp(const Pixel& in, const Pixel& low, const Pixel& high)
{
  return max(low, min(high, in));
}

//------------------------------------------------------------------------------
__global__ void k_clampOcclusion(
    float* occ, size_t stride, uint32_t width, uint32_t height,
    Texture2D occ_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x /*+ roi_x*/;
  const int y = blockIdx.y*blockDim.y + threadIdx.y /*+ roi_y*/;
  if (x<width && y<height)
  {
    occ[y*stride+x] = 1.f - clamp(tex2DFetch<float>(occ_tex, x, y)-1.f, 0.f, 1.f);
  }
}

//------------------------------------------------------------------------------
void occlusionCandidatesUniqunessMapping(
    ImageGpu32fC1::Ptr occ,
    const ImageGpu32fC1::Ptr& disp)
{
  occ->setValue(0.0f);
  std::shared_ptr<ze::cu::Texture2D> disp_tex = disp->genTexture();
  ze::cu::Fragmentation<> frag(disp->size());
  k_occlusionCandidatesUniqunessMapping
      <<<
        frag.dimGrid, frag.dimBlock
      >>> (occ->cuData(), occ->stride(), occ->width(), occ->height(),
           *disp_tex);

  // clamp the occlusions to get a nice mask with 0 and 1 entries
  std::shared_ptr<Texture2D> occ_tex = occ->genTexture();
  k_clampOcclusion
      <<<
        frag.dimGrid, frag.dimBlock
      >>> (occ->cuData(), occ->stride(), occ->width(), occ->height(), *occ_tex);
}

} // namespace cu
} // namespace ze
