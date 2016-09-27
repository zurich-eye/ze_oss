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
#ifndef IMP_CU_K_STEREO_CTF_WARPING_LEVEL_HUBER_CUH
#define IMP_CU_K_STEREO_CTF_WARPING_LEVEL_HUBER_CUH

#include <cuda_runtime_api.h>
#include <imp/core/types.hpp>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_k_derivative.cuh>
#include <imp/cuda_toolkit/helper_math.hpp>


namespace ze {
namespace cu {


//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_preconditioner(Pixel* xi, size_t stride,
                                 uint32_t width, uint32_t height,
                                 // uint32_t roi_x, uint32_t roi_y,
                                 float lambda, Texture2D ix_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x<width && y<height)
  {
    Pixel ix;
    tex2DFetch(ix, ix_tex, x, y);
    xi[y*stride+x] = 4 + fabs(lambda*ix);
  }
}

//-----------------------------------------------------------------------------
/** restricts the udpate to +/- lin_step around the given value in lin_tex
 * @note \a d_srcdst and the return value is identical.
 * @todo (MWE) move function to common kernel def file for all stereo models
 */
template<typename Pixel>
__device__ Pixel k_linearized_update(Pixel& d_srcdst, Texture2D& lin_tex,
                                     const float lin_step,
                                     const int x, const int y)
{
  Pixel lin = tex2DFetch<Pixel>(lin_tex, x, y);
  d_srcdst = max(lin-lin_step,
                 min(lin+lin_step, d_srcdst));
  return d_srcdst;
}

//-----------------------------------------------------------------------------
/**
 * @brief k_primalUpdate is the Huber-L1-Precondition model's primal update kernel
 * @note PPixel and DPixel denote for the Pixel type/dimension of primal and dual variable
 */
template<typename PPixel>
__global__ void k_primalUpdate(PPixel* d_u, PPixel* d_u_prev, const size_t stride,
                               uint32_t width, uint32_t height,
                               const float lambda, const float tau,
                               const float lin_step,
                               Texture2D u_tex, Texture2D u0_tex,
                               Texture2D pu_tex, Texture2D q_tex,
                               Texture2D ix_tex, Texture2D xi_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x<width && y<height)
  {
    float u_prev = tex2DFetch<float>(u_tex, x, y);
    float q = tex2DFetch<float>(q_tex, x, y);
    float ix = tex2DFetch<float>(ix_tex, x, y);
    float xi = tex2DFetch<float>(xi_tex, x, y);

    float div = dpAd(pu_tex, x, y, width, height);

    float u = u_prev - tau/xi * (-div + lambda*ix*q);

    u = k_linearized_update(u, u0_tex, lin_step, x, y);
    d_u[y*stride+x] = u;
    d_u_prev[y*stride+x] = 2.f*u - u_prev;
  }
}

//-----------------------------------------------------------------------------
/**
 * @brief k_primalUpdate is the Huber-L1-Precondition model's primal update kernel
 * @note PPixel and DPixel denote for the Pixel type/dimension of primal and dual variable
 */
template<typename PPixel>
__global__ void k_primalUpdate(PPixel* d_u, PPixel* d_u_prev, const size_t stride,
                               uint32_t width, uint32_t height,
                               const float tau, const float lin_step,
                               Texture2D lambda_tex,
                               Texture2D u_tex, Texture2D u0_tex,
                               Texture2D pu_tex, Texture2D q_tex,
                               Texture2D ix_tex, Texture2D xi_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x<width && y<height)
  {
    float u_prev = tex2DFetch<float>(u_tex, x, y);
    float q = tex2DFetch<float>(q_tex, x, y);
    float ix = tex2DFetch<float>(ix_tex, x, y);
    float xi = tex2DFetch<float>(xi_tex, x, y);

    float div = dpAd(pu_tex, x, y, width, height);

    float lambda = tex2DFetch<float>(lambda_tex, x,y);
    float u = u_prev - tau/xi * (-div + lambda*ix*q);

    u = k_linearized_update(u, u0_tex, lin_step, x, y);
    d_u[y*stride+x] = u;
    d_u_prev[y*stride+x] = 2.f*u - u_prev;
  }
}

//-----------------------------------------------------------------------------
template<typename PPixel, typename DPixel>
__global__ void k_dualUpdate(DPixel* d_pu, const size_t stride_pu,
                             PPixel* d_q, const size_t stride_q,
                             uint32_t width, uint32_t height,
                             const float lambda, const float eps_u,
                             const float sigma, const float eta,
                             Texture2D u_prev_tex, Texture2D u0_tex,
                             Texture2D pu_tex, Texture2D q_tex,
                             Texture2D ix_tex, Texture2D it_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;
  if (x<width && y<height)
  {
    const float sigma_by_eta = sigma/eta;

    // update pu
    float2 du = dp(u_prev_tex, x, y);
    float2 pu = tex2DFetch<float2>(pu_tex, x,y);
    pu  = (pu + sigma_by_eta*du) / (1.f + sigma_by_eta*eps_u);
    pu = pu / max(1.0f, length(pu));
    d_pu[y*stride_pu+x] = {pu.x, pu.y};

    // update q
    float u_prev = tex2DFetch<float>(u_prev_tex, x, y);
    float u0 = tex2DFetch<float>(u0_tex, x, y);
    float q = tex2DFetch<float>(q_tex, x, y);
    float ix = tex2DFetch<float>(ix_tex, x, y);
    float it = tex2DFetch<float>(it_tex, x, y);
    const float sigma_q = sigma / max(1e-6f, lambda * fabs(ix));
    q = q + lambda*sigma_q * (it + ix*(u_prev-u0));
    d_q[y*stride_q+x] = max(-1.f, min(1.f, q));
  }
}

//-----------------------------------------------------------------------------
template<typename PPixel, typename DPixel>
__global__ void k_dualUpdate(DPixel* d_pu, const size_t stride_pu,
                             PPixel* d_q, const size_t stride_q,
                             uint32_t width, uint32_t height,
                             const float eps_u,
                             const float sigma, const float eta,
                             Texture2D lambda_tex,
                             Texture2D u_prev_tex, Texture2D u0_tex,
                             Texture2D pu_tex, Texture2D q_tex,
                             Texture2D ix_tex, Texture2D it_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;
  if (x<width && y<height)
  {
    const float sigma_by_eta = sigma/eta;

    // update pu
    float2 du = dp(u_prev_tex, x, y);
    float2 pu = tex2DFetch<float2>(pu_tex, x,y);
    pu  = (pu + sigma_by_eta*du) / (1.f + sigma_by_eta*eps_u);
    pu = pu / max(1.0f, length(pu));
    d_pu[y*stride_pu+x] = {pu.x, pu.y};

    // update q
    float u_prev = tex2DFetch<float>(u_prev_tex, x, y);
    float u0 = tex2DFetch<float>(u0_tex, x, y);
    float q = tex2DFetch<float>(q_tex, x, y);
    float ix = tex2DFetch<float>(ix_tex, x, y);
    float it = tex2DFetch<float>(it_tex, x, y);
    float lambda = tex2DFetch<float>(lambda_tex, x,y);
    const float sigma_q = sigma / max(1e-6f, lambda * fabs(ix));
    q = q + lambda*sigma_q * (it + ix*(u_prev-u0));
    d_q[y*stride_q+x] = max(-1.f, min(1.f, q));
  }
}


} // namespace cu
} // namespace ze



#endif // IMP_CU_K_STEREO_CTF_WARPING_LEVEL_HUBER_CUH

