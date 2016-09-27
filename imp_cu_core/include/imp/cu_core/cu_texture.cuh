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
#ifndef IMP_CU_TEXTURE_CUH
#define IMP_CU_TEXTURE_CUH

#include <cuda_runtime.h>
#include <imp/cu_core/cu_pixel_conversion.hpp>
#include <imp/cu_core/cu_texture2d.cuh>

namespace ze {
namespace cu {

//-----------------------------------------------------------------------------
__device__ __forceinline__ void tex2DFetch(
    ze::Pixel8uC1& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  texel = ze::Pixel8uC1(::tex2D<uchar1>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f).x);
}

__device__ __forceinline__ void tex2DFetch(
    ze::Pixel8uC2& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  uchar2 val = ::tex2D<uchar2>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f);
  texel = ze::Pixel8uC2(val.x, val.y);
}

__device__ __forceinline__ void tex2DFetch(
    ze::Pixel8uC4& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  uchar4 val = ::tex2D<uchar4>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f);
  texel = ze::Pixel8uC4(val.x, val.y, val.z, val.w);
}

//-----------------------------------------------------------------------------
__device__ __forceinline__ void tex2DFetch(
    ze::Pixel16uC1& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  texel = ze::Pixel16uC1(::tex2D<ushort1>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f).x);
}

__device__ __forceinline__ void tex2DFetch(
    ze::Pixel16uC2& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  ushort2 val = ::tex2D<ushort2>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f);
  texel = ze::Pixel16uC2(val.x, val.y);
}

__device__ __forceinline__ void tex2DFetch(
    ze::Pixel16uC4& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  ushort4 val = ::tex2D<ushort4>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f);
  texel = ze::Pixel16uC4(val.x, val.y, val.z, val.w);
}

//-----------------------------------------------------------------------------
__device__ __forceinline__ void tex2DFetch(
    ze::Pixel32sC1& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  texel = ze::Pixel32sC1(::tex2D<int1>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f).x);
}

__device__ __forceinline__ void tex2DFetch(
    ze::Pixel32sC2& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  int2 val = ::tex2D<int2>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f);
  texel = ze::Pixel32sC2(val.x, val.y);
}

__device__ __forceinline__ void tex2DFetch(
    ze::Pixel32sC4& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  int4 val = ::tex2D<int4>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f);
  texel = ze::Pixel32sC4(val.x, val.y, val.z, val.w);
}

//-----------------------------------------------------------------------------
__device__ __forceinline__ void tex2DFetch(
    ze::Pixel32fC1& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  texel = ze::Pixel32fC1(::tex2D<float1>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f).x);
}

__device__ __forceinline__ void tex2DFetch(
    ze::Pixel32fC2& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  float2 val = ::tex2D<float2>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f);
  texel = ze::Pixel32fC2(val.x, val.y);
}

__device__ __forceinline__ void tex2DFetch(
    ze::Pixel32fC4& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  float4 val = ::tex2D<float4>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f);
  texel = ze::Pixel32fC4(val.x, val.y, val.z, val.w);
}

//-----------------------------------------------------------------------------
__device__ __forceinline__ void tex2DFetch(
    float& texel, const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  texel = ::tex2D<float>(tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f);
}

//-----------------------------------------------------------------------------
template<typename T>
__device__ __forceinline__
T tex2DFetch(
    const Texture2D& tex, float x, float y,
    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
{
  return ::tex2D<T>(
        tex.tex_object, x*mul_x+add_x+0.5f, y*mul_y+add_y+.5f);
}

////-----------------------------------------------------------------------------
//template<typename T>
//__device__ __forceinline__
//typename std::enable_if<!std::is_integral<T>::value && !std::is_floating_point<T>::value, T>::type
//tex2DFetch(
//    const Texture2D& tex, float x, float y,
//    float mul_x=1.f, float mul_y=1.f, float add_x=0.f, float add_y=0.f)
//{
//  T val;
//  tex2DFetch(val, tex, x, y, mul_x, mul_y, add_x, add_y);
//  return val;
//}



} // namespace cu
} // namespace ze

#endif // IMP_CU_TEXTURE_CUH

