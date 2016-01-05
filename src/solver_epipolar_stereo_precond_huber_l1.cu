#include <imp/cu_correspondence/solver_epipolar_stereo_precond_huber_l1.cuh>

#include <cuda_runtime.h>

#include <glog/logging.h>

#include <imp/cu_correspondence/variational_stereo_parameters.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_imgproc/cu_image_filter.cuh>
#include <imp/cu_imgproc/cu_resample.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_core/cu_math.cuh>
#include <imp/cu_core/cu_k_setvalue.cuh>
#include <imp/cu_imgproc/edge_detectors.cuh>

#include "warped_gradients_kernel.cuh"
#include "solver_precond_huber_l1_kernel.cuh"
#include "solver_stereo_precond_huber_l1_weighted_kernel.cuh"
//#include "solver_epipolar_stereo_precond_huber_l1_kernel.cuh"

#define USE_EDGES 1

namespace imp {
namespace cu {

//------------------------------------------------------------------------------
SolverEpipolarStereoPrecondHuberL1::~SolverEpipolarStereoPrecondHuberL1()
{
  // thanks to smart pointers
}

//------------------------------------------------------------------------------
SolverEpipolarStereoPrecondHuberL1::SolverEpipolarStereoPrecondHuberL1(
    const std::shared_ptr<Parameters>& params, imp::Size2u size, size_t level,
    const std::vector<cu::PinholeCamera>& cams,
    const cu::Matrix3f& F,
    const cu::SE3<float>& T_mov_fix,
    const imp::cu::ImageGpu32fC1& depth_proposal,
    const imp::cu::ImageGpu32fC1& depth_proposal_sigma2)
  : SolverStereoAbstract(params, size, level)
{
  u_.reset(new ImageGpu32fC1(size));
  u_prev_.reset(new ImageGpu32fC1(size));
  u0_.reset(new ImageGpu32fC1(size));
  pu_.reset(new ImageGpu32fC2(size));
  q_.reset(new ImageGpu32fC1(size));
  iw_.reset(new ImageGpu32fC1(size));
  ix_.reset(new ImageGpu32fC1(size));
  it_.reset(new ImageGpu32fC1(size));
  xi_.reset(new ImageGpu32fC1(size));
  g_.reset(new ImageGpu32fC1(size));

  depth_proposal_.reset(new ImageGpu32fC1(size));
  depth_proposal_sigma2_.reset(new ImageGpu32fC1(size));

  u_tex_ = u_->genTexture(false, cudaFilterModeLinear);
  u_prev_tex_ =  u_prev_->genTexture(false, cudaFilterModeLinear);
  u0_tex_ =  u0_->genTexture(false, cudaFilterModeLinear);
  pu_tex_ =  pu_->genTexture(false, cudaFilterModeLinear);
  q_tex_ =  q_->genTexture(false, cudaFilterModeLinear);
  ix_tex_ =  ix_->genTexture(false, cudaFilterModeLinear);
  it_tex_ =  it_->genTexture(false, cudaFilterModeLinear);
  xi_tex_ =  xi_->genTexture(false, cudaFilterModeLinear);
  g_tex_ =  g_->genTexture(false, cudaFilterModeLinear);
  depth_proposal_tex_ =  depth_proposal_->genTexture(false, cudaFilterModeLinear);
  depth_proposal_sigma2_tex_ =  depth_proposal_sigma2_->genTexture(false, cudaFilterModeLinear);


  float scale_factor = std::pow(params->ctf.scale_factor, level);

  if (depth_proposal.size() == size)
  {
    LOG(INFO) << "Copy depth proposals " << depth_proposal.size() << " to level0 "
              << depth_proposal_->size();
    depth_proposal.copyTo(*depth_proposal_);
    depth_proposal_sigma2.copyTo(*depth_proposal_sigma2_);
  }
  else
  {
    float downscale_factor = 0.5f*((float)size.width()/(float)depth_proposal.width()+
                                   (float)size.height()/(float)depth_proposal.height());

    if (params_->verbose>2)
      LOG(INFO) << "depth proposal downscaled to level: " << level << "; size: " << size
                << "; downscale_factor: " << downscale_factor;

    imp::cu::resample(*depth_proposal_, depth_proposal);
    imp::cu::resample(*depth_proposal_sigma2_, depth_proposal_sigma2);
  }

  F_ = F;
  T_mov_fix_ = T_mov_fix;

  // assuming we receive the camera matrix for level0
  if  (level == 0)
  {
    cams_ = cams;
  }
  else
  {
    for (auto cam : cams)
    {
      cu::PinholeCamera scaled_cam = cam * scale_factor;
      cams_.push_back(scaled_cam);
    }
  }
}

//------------------------------------------------------------------------------
void SolverEpipolarStereoPrecondHuberL1::init()
{
  u_->setValue(0.0f);
  pu_->setValue(0.0f);
  q_->setValue(0.0f);
  // other variables are init and/or set when needed!
}

//------------------------------------------------------------------------------
void SolverEpipolarStereoPrecondHuberL1::init(const SolverStereoAbstract& rhs)
{
  const SolverEpipolarStereoPrecondHuberL1* from =
      dynamic_cast<const SolverEpipolarStereoPrecondHuberL1*>(&rhs);

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
  imp::cu::resample(*q_, *from->q_, imp::InterpolationMode::point, false);
}

//------------------------------------------------------------------------------
void SolverEpipolarStereoPrecondHuberL1::solve(std::vector<ImageGpu32fC1::Ptr> images)
{
  if (params_->verbose > 0)
    std::cout << "SolverEpipolarStereoPrecondHuberL1: solving level " << level_ << " with " << images.size() << " images" << std::endl;

  // sanity check:
  // TODO


  // image textures
  i1_tex_ = images.at(0)->genTexture(false, cudaFilterModeLinear);
  i2_tex_ = images.at(1)->genTexture(false, cudaFilterModeLinear);


  // constants
  constexpr float tau = 0.95f;
  constexpr float sigma = 0.95f;
  float lin_step = 0.5f;
  Fragmentation<> frag(size_);
  constexpr float eta = 2.0f;

  // init
  u_->copyTo(*u_prev_);


  // check if a pointwise lambda is set in the parameters. otherwise we create
  // a local one to simplify kernel interfaces
  cu::ImageGpu32fC1::Ptr lambda;
  if (params_->lambda_pointwise)
    lambda = params_->lambda_pointwise;
  else
  {
    // make it as small as possible to reduce memory overhead. access is then
    // handled by the texture
    lambda.reset(new ImageGpu32fC1(1,1));
    lambda->setValue(params_->lambda);
  }
  lambda_tex_ = lambda->genTexture(false,cudaFilterModePoint,
                                   cudaAddressModeClamp, cudaReadModeElementType);

  // compute edge weight
  imp::cu::naturalEdges(*g_, *images.at(0),
                        params_->edge_sigma, params_->edge_alpha, params_->edge_q);

  // warping
  for (std::uint32_t warp = 0; warp < params_->ctf.warps; ++warp)
  {
    if (params_->verbose > 5)
      std::cout << "SOLVING warp iteration of Huber-L1 stereo model." << std::endl;

    u_->copyTo(*u0_);


    // compute warped spatial and temporal gradients
    k_warpedGradientsEpipolarConstraint
        <<<
          frag.dimGrid, frag.dimBlock
        >>> (iw_->data(), ix_->data(), it_->data(), ix_->stride(), ix_->width(), ix_->height(),
             cams_.at(0), cams_.at(1), F_, T_mov_fix_,
             *i1_tex_, *i2_tex_, *u0_tex_,
             *depth_proposal_tex_);

    // compute preconditioner
#if USE_EDGES
    // compute preconditioner
    k_preconditionerWeighted
        <<<
          frag.dimGrid, frag.dimBlock
        >>> (xi_->data(), xi_->stride(), xi_->width(), xi_->height(),
             *lambda_tex_, *ix_tex_, *g_tex_);
#else
    k_preconditioner
        <<<
          frag.dimGrid, frag.dimBlock
        >>> (xi_->data(), xi_->stride(), xi_->width(), xi_->height(),
             params_->lambda, *ix_tex_);
#endif

    for (std::uint32_t iter = 0; iter < params_->ctf.iters; ++iter)
    {
#if USE_EDGES
      // dual update kernel
      k_dualUpdateWeighted
          <<<
            frag.dimGrid, frag.dimBlock
          >>> (pu_->data(), pu_->stride(), q_->data(), q_->stride(),
               size_.width(), size_.height(),
               params_->eps_u, sigma, eta, *lambda_tex_,
               *u_prev_tex_, *u0_tex_, *pu_tex_, *q_tex_, *ix_tex_, *it_tex_, *g_tex_);

      // and primal update kernel
      k_primalUpdateWeighted
          <<<
            frag.dimGrid, frag.dimBlock
          >>> (u_->data(), u_prev_->data(), u_->stride(),
               size_.width(), size_.height(),
               tau, lin_step, *lambda_tex_,
               *u_tex_, *u0_tex_, *pu_tex_, *q_tex_, *ix_tex_, *xi_tex_, *g_tex_);
#else
      // dual update kernel
      k_dualUpdate
          <<<
            frag.dimGrid, frag.dimBlock
          >>> (pu_->data(), pu_->stride(), q_->data(), q_->stride(),
               size_.width(), size_.height(),
               params_->eps_u, sigma, eta, *lambda_tex_,
               *u_prev_tex_, *u0_tex_, *pu_tex_, *q_tex_, *ix_tex_, *it_tex_);

      // and primal update kernel
      k_primalUpdate
          <<<
            frag.dimGrid, frag.dimBlock
          >>> (u_->data(), u_prev_->data(), u_->stride(),
               size_.width(), size_.height(),
               tau, lin_step, *lambda_tex_,
               *u_tex_, *u0_tex_, *pu_tex_, *q_tex_, *ix_tex_, *xi_tex_);
#endif
    } // iters
    lin_step /= 1.2f;

  } // warps



  IMP_CUDA_CHECK();
}



} // namespace cu
} // namespace imp

