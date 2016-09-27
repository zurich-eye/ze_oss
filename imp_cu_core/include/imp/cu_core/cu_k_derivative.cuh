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
#ifndef IMP_CU_K_DERIVATIVE_CUH
#define IMP_CU_K_DERIVATIVE_CUH

#include <cuda_runtime.h>
#include <imp/cu_core/cu_texture.cuh>

namespace ze {
namespace cu {

//-----------------------------------------------------------------------------
/** compute forward differences in x- and y- direction */
static __device__ __forceinline__ float2 dp(
    const ze::cu::Texture2D& tex, size_t x, size_t y)
{
  float2 grad = make_float2(0.0f, 0.0f);
  float cval = tex2DFetch<float>(tex, x, y);
  grad.x = tex2DFetch<float>(tex, x+1.f, y) - cval;
  grad.y = tex2DFetch<float>(tex, x, y+1.f) - cval;
  return grad;
}

//-----------------------------------------------------------------------------
/** compute divergence using backward differences (adjugate from dp). */
static __device__ __forceinline__
float dpAd(const ze::cu::Texture2D& tex,
           size_t x, size_t y, size_t width, size_t height)
{
  float2 c = tex2DFetch<float2>(tex, x,y);
  float2 w = tex2DFetch<float2>(tex, x-1.f, y);
  float2 n = tex2DFetch<float2>(tex, x, y-1.f);

  if (x == 0)
    w.x = 0.0f;
  else if (x >= width-1)
    c.x = 0.0f;

  if (y == 0)
    n.y = 0.0f;
  else if (y >= height-1)
    c.y = 0.0f;

  return (c.x - w.x + c.y - n.y);
}



//-----------------------------------------------------------------------------
/** compute forward differences in x- and y- direction */
static __device__ __forceinline__ float2 dpWeighted(
    const ze::cu::Texture2D& tex, const ze::cu::Texture2D& g_tex,
    size_t x, size_t y)
{
  float cval = tex2DFetch<float>(tex, x, y);
  float g = tex2DFetch<float>(g_tex, x, y);

  return make_float2(
        g*(tex2DFetch<float>(tex, x+1.f, y) - cval),
        g*(tex2DFetch<float>(tex, x, y+1.f) - cval));
}

//-----------------------------------------------------------------------------
/** compute weighted divergence using backward differences (adjugate from dp). */
static __device__ __forceinline__
float dpAdWeighted(const ze::cu::Texture2D& tex, const ze::cu::Texture2D& g_tex,
                   size_t x, size_t y, size_t width, size_t height)
{
  float2 c = tex2DFetch<float2>(tex, x, y);
  float2 w = tex2DFetch<float2>(tex, x-1.f, y);
  float2 n = tex2DFetch<float2>(tex, x, y-1);

  float g = tex2DFetch<float>(g_tex, x, y);
  float g_w = tex2DFetch<float>(g_tex, x-1.f, y);
  float g_n = tex2DFetch<float>(g_tex, x, y-1);

  if (x == 0)
    w.x = 0.0f;
  else if (x >= width-1)
    c.x = 0.0f;

  if (y == 0)
    n.y = 0.0f;
  else if (y >= height-1)
    c.y = 0.0f;

  return (c.x*g - w.x*g_w + c.y*g - n.y*g_n);
}

//-----------------------------------------------------------------------------
/** compute directed divergence using backward differences (adjugate from dp). */
static __device__ __forceinline__
float dpAdDirected(const ze::cu::Texture2D& tex,
                   const ze::cu::Texture2D& tensor_a_tex,
                   const ze::cu::Texture2D& tensor_b_tex,
                   const ze::cu::Texture2D& tensor_c_tex,
                   size_t x, size_t y, size_t width, size_t height)
{
  float2 c = tex2DFetch<float2>(tex, x, y);
  float2 w = tex2DFetch<float2>(tex, x-1.f, y);
  float2 n = tex2DFetch<float2>(tex, x, y-1);

  float tensor_a   = tex2DFetch<float>(tensor_a_tex, x,     y);
  float tensor_a_w = tex2DFetch<float>(tensor_a_tex, x-1.f, y);
  float tensor_b   = tex2DFetch<float>(tensor_b_tex, x,     y);
  float tensor_b_n = tex2DFetch<float>(tensor_b_tex, x    , y-1.f);
  float tensor_c   = tex2DFetch<float>(tensor_c_tex, x,     y);
  float tensor_c_w = tex2DFetch<float>(tensor_c_tex, x-1.f, y);
  float tensor_c_n = tex2DFetch<float>(tensor_c_tex, x    , y-1.f);

  if (x == 0)
    w = make_float2(0.0f, 0.0f);
  else if (x >= width-1)
    c.x = 0.0f;

  if (y == 0)
    n = make_float2(0.0f, 0.0f);
  else if (y >= height-1)
    c.y = 0.0f;

  return ((c.x*tensor_a + c.y*tensor_c) - (w.x*tensor_a_w + w.y*tensor_c_w) +
          (c.x*tensor_c + c.y*tensor_b) - (n.x*tensor_c_n + n.y*tensor_b_n));
}

//-----------------------------------------------------------------------------
/** compute directed divergence using backward differences (adjugate from dp). */
static __device__ __forceinline__
float dpAdTensor(const ze::cu::Texture2D& tex, const ze::cu::Texture2D& g_tex,
                 size_t x, size_t y, size_t width, size_t height)
{
  float2 c = tex2DFetch<float2>(tex, x, y);
  float2 w = tex2DFetch<float2>(tex, x-1.f, y);
  float2 n = tex2DFetch<float2>(tex, x, y-1);

  float2 g = tex2DFetch<float2>(g_tex, x, y);
  float2 g_w = tex2DFetch<float2>(g_tex, x-1.f, y);
  float2 g_n = tex2DFetch<float2>(g_tex, x, y-1);

  if (x == 0)
    w.x = 0.0f;
  else if (x >= width-1)
    c.x = 0.0f;

  if (y == 0)
    n.y = 0.0f;
  else if (y >= height-1)
    c.y = 0.0f;

  return (c.x*g.x - w.x*g_w.x + c.y*g.y - n.y*g_n.y);
}

} // namespace cu
} // namespace ze

#endif // IMP_CU_K_DERIVATIVE_CUH

