#ifndef IMP_CU_K_STEREO_CTF_WARPING_LEVEL_HUBER_CUH
#define IMP_CU_K_STEREO_CTF_WARPING_LEVEL_HUBER_CUH

#include <cuda_runtime_api.h>
#include <imp/core/types.hpp>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_k_derivative.cuh>
#include <imp/cuda_toolkit/helper_math.h>


namespace imp {
namespace cu {


//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_preconditioner(Pixel* xi, size_t stride,
                                 std::uint32_t width, std::uint32_t height,
                                 // std::uint32_t roi_x, std::uint32_t roi_y,
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
                               std::uint32_t width, std::uint32_t height,
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
                               std::uint32_t width, std::uint32_t height,
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
                             std::uint32_t width, std::uint32_t height,
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
                             std::uint32_t width, std::uint32_t height,
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
} // namespace imp



#endif // IMP_CU_K_STEREO_CTF_WARPING_LEVEL_HUBER_CUH

