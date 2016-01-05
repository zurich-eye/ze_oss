#include <imp/cu_imgproc/cu_tvl1_denoising.cuh>

#include <iostream>

#include <cuda_runtime.h>

#include <imp/cuda_toolkit/helper_math.h>
#include <imp/core/pixel.hpp>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_core/cu_k_derivative.cuh>
#include <imp/cu_core/cu_math.cuh>


namespace imp {
namespace cu {

//-----------------------------------------------------------------------------
__global__ void k_initTvL1Solver(Pixel32fC1* d_u, Pixel32fC1* d_u_prev, size_t stride_u,
                                 Pixel32fC2* d_p, size_t stride_p,
                                 imp::cu::Texture2D f_tex,
                                 size_t width, size_t height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x<width && y<height)
  {
    float val = tex2D<float>(f_tex, x+.5f, y+.5f);
    d_u[y*stride_u + x] = val;
    d_u_prev[y*stride_u + x] = val;
    d_p[y*stride_p + x] = Pixel32fC2(0.0f, 0.0f);
  }
}

//-----------------------------------------------------------------------------
__global__ void k_tvL1PrimalUpdate(
    Pixel32fC1* d_u, Pixel32fC1* d_u_prev, size_t stride_u,
    Texture2D f_tex, Texture2D u_tex, Texture2D p_tex,
    float lambda, float tau, float theta, size_t width, size_t height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x<width && y<height)
  {
    float f = tex2D<float>(f_tex, x+.5f, y+.5f);
    float u = tex2D<float>(u_tex, x+.5f, y+.5f);
    float u_prev = u;
    float div = dpAd(p_tex, x, y, width, height);
    u += tau*div;

    float tau_lambda = tau*lambda;
    float residual = u - f;
    if (residual < -tau_lambda)
    {
      u += tau_lambda;
    }
    else if (residual > tau_lambda)
    {
      u -= tau_lambda;
    }
    else
    {
      u = f;
    }

    d_u[y*stride_u + x] = u;
    d_u_prev[y*stride_u + x] = u + theta*(u-u_prev);
  }
}

//-----------------------------------------------------------------------------
__global__ void k_tvL1DualUpdate(
    Pixel32fC2* d_p, size_t stride_p, Texture2D p_tex, Texture2D u_prev_tex,
    float sigma, size_t width, size_t height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x<width && y<height)
  {
    float2 p = tex2D<float2>(p_tex, x+.5f, y+.5f);
    float2 dp_u = dp(u_prev_tex, x, y);

    p = p + sigma*dp_u;
    p = p / max(1.0f, length(p));
    d_p[y*stride_p + x] = {p.x, p.y};
  }
}

//-----------------------------------------------------------------------------
//! @todo (MWE) move to a common place (also needed for other algorithms!)
__global__ void k_tvL1convertResult8uC1(Pixel8uC1* d_u, size_t stride_u,
                                    imp::cu::Texture2D u_tex,
                                    size_t width, size_t height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x<width && y<height)
  {
    d_u[y*stride_u + x] = static_cast<std::uint8_t>(
          255.0f * tex2D<float>(u_tex, x+.5f, y+.5f));
  }
}

//#############################################################################

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void TvL1Denoising<Pixel, pixel_type>::init(const Size2u& size)
{
  Base::init(size);
  IMP_CUDA_CHECK();

  // setup textures
  f_tex_ = f_->genTexture(false, cudaFilterModeLinear, cudaAddressModeClamp,
                          (f_->bitDepth()==8) ? cudaReadModeNormalizedFloat :
                                                cudaReadModeElementType);
  u_tex_ = u_->genTexture(false, cudaFilterModeLinear, cudaAddressModeClamp,
                          cudaReadModeElementType);
  u_prev_tex_ = u_prev_->genTexture(false, cudaFilterModeLinear,
                                    cudaAddressModeClamp, cudaReadModeElementType);
  p_tex_ = p_->genTexture(false, cudaFilterModeLinear, cudaAddressModeClamp,
                          cudaReadModeElementType);
  IMP_CUDA_CHECK();

  // init internal vars
  k_initTvL1Solver
      <<< dimGrid(), dimBlock() >>> (u_->data(), u_prev_->data(), u_->stride(),
                                     p_->data(), p_->stride(),
                                     *f_tex_, size_.width(), size_.height());
  IMP_CUDA_CHECK();
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void TvL1Denoising<Pixel, pixel_type>::denoise(const ImageBase::Ptr& dst,
                                               const ImageBase::Ptr& src)
{
  if (params_.verbose)
  {
    std::cout << "[Solver @gpu] TvL1Denoising::denoise:" << std::endl;
  }

  if (src->size() != dst->size())
  {
    throw imp::cu::Exception("Input and output image are not of the same size.",
                             __FILE__, __FUNCTION__, __LINE__);
  }

  f_ = std::dynamic_pointer_cast<ImageGpu>(src);
  //! @todo (MWE) we could use dst for u_ if pixel_type is consistent

  if (size_ != f_->size())
  {
    this->init(f_->size());
  }

  // internal params
  float theta = 1.0f;
  //float L = sqrtf(8.0f);
  float sigma = 1.f/sqrtf(8.0f);
  float tau = 1.f/8.f;

  for(int iter = 0; iter < this->params_.max_iter; ++iter)
  {
    if (sigma < 1000.0f)
      theta = 1.f/sqrtf(1.0f+0.7f*this->params_.lambda*tau);
    else
      theta = 1.0f;

    if (params_.verbose)
    {
      std::cout << "(TvL1 solver) iter: " << iter << "; tau: " << tau
                << "; sigma: " << sigma << "; theta: " << theta << std::endl;
    }

    k_tvL1DualUpdate
        <<< dimGrid(), dimBlock() >>> (p_->data(), p_->stride(),
                                       *p_tex_, *u_prev_tex_,
                                       sigma, size_.width(), size_.height());

    k_tvL1PrimalUpdate
        <<< dimGrid(), dimBlock() >>> (u_->data(), u_prev_->data(), u_->stride(),
                                       *f_tex_, *u_tex_, *p_tex_,
                                       params_.lambda, tau, theta,
                                       size_.width(), size_.height());

    sigma /= theta;
    tau *= theta;
  }
  IMP_CUDA_CHECK();

  switch (dst->pixelType())
  {
  case PixelType::i8uC1:
  {
    std::shared_ptr<ImageGpu8uC1> u(std::dynamic_pointer_cast<ImageGpu8uC1>(dst));
    k_tvL1convertResult8uC1
        <<< dimGrid(), dimBlock() >>> (u->data(), u->stride(),
                                       *u_tex_, size_.width(), size_.height());
  }
  break;
  case PixelType::i32fC1:
  {
    std::shared_ptr<ImageGpu32fC1> u(std::dynamic_pointer_cast<ImageGpu32fC1>(dst));
    u_->copyTo(*u);
  }
  break;
  default:
    throw imp::cu::Exception("Unsupported PixelType.",
                             __FILE__, __FUNCTION__, __LINE__);
  }
  IMP_CUDA_CHECK();
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void TvL1Denoising<Pixel, pixel_type>::print(std::ostream& os) const
{
  os << "TvL1 Denoising:" << std::endl;
  this->Base::print(os);
}

//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class TvL1Denoising<imp::Pixel8uC1, imp::PixelType::i8uC1>;
template class TvL1Denoising<imp::Pixel32fC1, imp::PixelType::i32fC1>;

} // namespace cu
} // namespace imp
