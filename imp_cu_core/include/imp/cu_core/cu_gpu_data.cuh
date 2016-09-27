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
#ifndef IMP_CU_GPU_DATA_CUH
#define IMP_CU_GPU_DATA_CUH

#include <imp/core/types.hpp>
#include <imp/core/roi.hpp>


namespace imp { namespace cu {

template<typename Pixel>
struct GpuData2D
{
  Pixel* data;
  std::uint32_t width;
  std::uint32_t height;
  imp::Roi2u roi;
  size_t stride;

  __host__ __device__
  GpuData2D(Pixel* _data, size_t _stride,
            std::uint32_t _width, std::uint32_t _height,
            imp::Roi2u _roi=imp::Roi2u())
    : data(_data)
    , stride(_stride)
    , width(_width)
    , height(_height)
    //, roi((_roi == imp::Roi2u()) ? imp::Roi2u(0u, 0u, _width, _height) : _roi)
  {
  }

  bool valid() { return (data != nullptr && width>0 && height >0); }

  void roiReset() { roi = imp::Roi2u(0,0,width,height); }

  __host__ __device__ __forceinline__
  bool inRoi(int x, int y)
  {
    if (roi != imp::Roi2u())
      return (x>=roi.x() && y>=roi.y() && x<roi.width() && y<roi.height());
    else
      return inBounds(x,y);
  }

  __host__ __device__ __forceinline__
  bool inBounds(int x, int y)
  {
    return (x>=0 && y>=0 && x<width && y<height);
  }

  __host__ __device__ __forceinline__
  size_t pitch()
  {
    return stride*sizeof(Pixel);
  }

  __host__ __device__ __forceinline__
  Pixel operator()(int x, int y)
  {
    return data[y*stride+x];
  }

  __host__ __device__ __forceinline__
  Pixel operator[](int x)
  {
    return data[x];
  }
};

} // namespace cu
} // namespace imp

#endif // IMP_CU_GPU_DATA_CUH

