#include <imp/cu_correspondence/solver_stereo_huber_l1.cuh>

#include <cmath>

#include <cuda_runtime.h>

#include <imp/cu_correspondence/variational_stereo_parameters.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_imgproc/cu_image_filter.cuh>
#include <imp/cu_imgproc/cu_resample.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_core/cu_math.cuh>

#include "warped_gradients_kernel.cuh"
#include "solver_stereo_huber_l1_kernel.cuh"

namespace imp {
namespace cu {


//------------------------------------------------------------------------------
SolverStereoHuberL1::~SolverStereoHuberL1()
{
  // thanks to smart pointers
}

//------------------------------------------------------------------------------
SolverStereoHuberL1::SolverStereoHuberL1(
    const Parameters::Ptr& params,
    imp::Size2u size,
    size_t level)
  : SolverStereoAbstract(params, size, level)
{
  u_.reset(new ImageGpu32fC1(size));
  u_prev_.reset(new ImageGpu32fC1(size));
  u0_.reset(new ImageGpu32fC1(size));
  pu_.reset(new ImageGpu32fC2(size));
  ix_.reset(new ImageGpu32fC1(size));
  it_.reset(new ImageGpu32fC1(size));

  // and its textures
  u_tex_ = u_->genTexture(false, cudaFilterModeLinear);
  u_prev_tex_ =  u_prev_->genTexture(false, cudaFilterModeLinear);
  u0_tex_ =  u0_->genTexture(false, cudaFilterModeLinear);
  pu_tex_ =  pu_->genTexture(false, cudaFilterModeLinear);
  ix_tex_ =  ix_->genTexture(false, cudaFilterModeLinear);
  it_tex_ =  it_->genTexture(false, cudaFilterModeLinear);
}

//------------------------------------------------------------------------------
void SolverStereoHuberL1::init()
{
  u_->setValue(0.0f);
  pu_->setValue(0.0f);
  // other variables are init and/or set when needed!
}

//------------------------------------------------------------------------------
void SolverStereoHuberL1::init(const SolverStereoAbstract& rhs)
{
  const SolverStereoHuberL1* from =
      dynamic_cast<const SolverStereoHuberL1*>(&rhs);

  float inv_sf = 1./params_->ctf.scale_factor; // >1 for adapting prolongated disparities

  if(params_->ctf.apply_median_filter)
  {
    imp::cu::filterMedian3x3(*from->u0_, *from->u_);
    imp::cu::resample(*u_, *from->u0_, imp::InterpolationMode::point, false);
  }
  else
  {
    imp::cu::resample(*u_, *from->u_, imp::InterpolationMode::point, false);
  }
  *u_ *= inv_sf;

  imp::cu::resample(*pu_, *from->pu_, imp::InterpolationMode::point, false);
}

//------------------------------------------------------------------------------
void SolverStereoHuberL1::solve(std::vector<ImageGpu32fC1::Ptr> images)
{
  if (params_->verbose > 0)
    std::cout << "StereoCtFWarpingLevelPrecondHuberL1: solving level " << level_ << " with " << images.size() << " images" << std::endl;

  // sanity check:
  // TODO

  i1_tex_ = images.at(0)->genTexture(false, cudaFilterModeLinear);
  i2_tex_ = images.at(1)->genTexture(false, cudaFilterModeLinear);
  u_->copyTo(*u_prev_);
  Fragmentation<> frag(size_);

  // constants
  const float L = std::sqrt(8.f);
  const float tau = 1.f/L;
  const float sigma = 1.f/L;
  float lin_step = 0.5f;

  // warping
  for (std::uint32_t warp = 0; warp < params_->ctf.warps; ++warp)
  {
    if (params_->verbose > 5)
      std::cout << "SOLVING warp iteration of Huber-L1 stereo model." << std::endl;

    u_->copyTo(*u0_);

    // compute warped spatial and temporal gradients
    k_warpedGradients
        <<<
          frag.dimGrid, frag.dimBlock
        >>> (ix_->data(), it_->data(), ix_->stride(), ix_->width(), ix_->height(),
             *i1_tex_, *i2_tex_, *u0_tex_);

    for (std::uint32_t iter = 0; iter < params_->ctf.iters; ++iter)
    {
      // dual kernel
      k_dualUpdate
          <<<
            frag.dimGrid, frag.dimBlock
          >>> (pu_->data(), pu_->stride(),
               size_.width(), size_.height(),
               params_->eps_u, sigma,
               *u_prev_tex_, *pu_tex_);

      // and primal kernel
      k_primalUpdate
          <<<
            frag.dimGrid, frag.dimBlock
          >>> (u_->data(), u_prev_->data(), u_->stride(),
               size_.width(), size_.height(),
               params_->lambda, tau, lin_step,
               *u_tex_, *u0_tex_, *pu_tex_, *ix_tex_, *it_tex_);
    } // iters
    lin_step /= 1.2f;

  } // warps
  IMP_CUDA_CHECK();
}



} // namespace cu
} // namespace imp

