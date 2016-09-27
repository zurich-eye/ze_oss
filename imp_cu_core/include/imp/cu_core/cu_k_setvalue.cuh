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
#ifndef IMP_CU_SETVALUE_CUH
#define IMP_CU_SETVALUE_CUH

//#include <stdio.h>

#include <cuda_runtime_api.h>

#if 0
#include <imp/cu_core/cu_gpu_data.cuh>
#endif

namespace ze {
namespace cu {

//-----------------------------------------------------------------------------
/**
 * @brief
 */
#if 0
template<typename Pixel>
__global__ void k_setValue(GpuData2D<Pixel>* dst, const Pixel value)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  // account for roi
  x+=dst->roi.x();
  y+=dst->roi.y();

  if (dst->inRoi(x,y))
  {
    dst->data[y*dst->stride+x] = value;
  }
}
#endif


//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_setValue(
    Pixel* d_dst, uint32_t offset, uint32_t roi_length, const Pixel value)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  if (x<roi_length)
  {
    d_dst[x+offset] = value;
  }
}


//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_setValue(Pixel* d_dst, size_t stride, const Pixel value,
                           size_t width, size_t height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x>=0 && y>=0 && x<width && y<height)
  {
    d_dst[y*stride+x] = value;
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel, typename T>
__global__ void k_pixelWiseMul(Pixel* d_dst, size_t stride, const T rhs,
                               size_t width, size_t height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x>=0 && y>=0 && x<width && y<height)
  {
    d_dst[y*stride+x] = d_dst[y*stride+x]*rhs;
  }
}


} // namespace cu
} // namespace ze

#endif // IMP_CU_SETVALUE_CUH
