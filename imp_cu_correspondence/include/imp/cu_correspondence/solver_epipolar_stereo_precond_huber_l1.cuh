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
#ifndef IMP_CU_EPIPOLAR_STEREO_PRECOND_HUBER_L1_CUH
#define IMP_CU_EPIPOLAR_STEREO_PRECOND_HUBER_L1_CUH

#include <cstdint>
#include <memory>

#include <imp/cu_correspondence/solver_stereo_abstract.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_matrix.cuh>
#include <imp/cu_core/cu_se3.cuh>
#include <imp/cu_core/cu_pinhole_camera.cuh>
#include <imp/core/size.hpp>

namespace ze {
namespace cu {

// forward decl
class VariationalStereoParameters;
class Texture2D;

/**
 * @brief The StereoCtFWarpingLevelPrecondHuberL1 class
 */
class SolverEpipolarStereoPrecondHuberL1 : public SolverStereoAbstract
{
public:
  SolverEpipolarStereoPrecondHuberL1() = delete;
  virtual ~SolverEpipolarStereoPrecondHuberL1();

  SolverEpipolarStereoPrecondHuberL1(const Parameters::Ptr& params,
                                     ze::Size2u size, size_t level,
                                     const std::vector<cu::PinholeCamera>& cams,
                                     const cu::Matrix3f& F,
                                     const cu::SE3<float>& T_mov_fix,
                                     const ImageGpu32fC1& depth_proposal,
                                     const ze::cu::ImageGpu32fC1& depth_proposal_sigma2);

  virtual void init();
  virtual void init(const SolverStereoAbstract& rhs);
  virtual inline void setFundamentalMatrix(const cu::Matrix3f& F) {F_ = F;}

  virtual void solve(std::vector<ImageGpu32fC1::Ptr> images);

  virtual inline ImageGpu32fC1::Ptr getDisparities() {return u_;}

protected:
  ImageGpu32fC1::Ptr u_; //!< disparities (result)
  std::unique_ptr<ImageGpu32fC1> u_prev_; //!< disparities results from previous iteration
  std::unique_ptr<ImageGpu32fC1> u0_; //!< disparities results from previous warp
  std::unique_ptr<ImageGpu32fC2> pu_; //!< dual variable for primal variable
  std::unique_ptr<ImageGpu32fC1> q_; //!< dual variable for data term
  std::unique_ptr<ImageGpu32fC1> iw_; //!< warped moving image
  std::unique_ptr<ImageGpu32fC1> ix_; //!< spatial gradients on moving (warped) image
  std::unique_ptr<ImageGpu32fC1> it_; //!< temporal gradients between warped and fixed image
  std::unique_ptr<ImageGpu32fC1> xi_; //!< preconditioner
  std::unique_ptr<ImageGpu32fC1> g_; //!< for edge weighting

  cu::Matrix3f F_;
  std::vector<cu::PinholeCamera> cams_;
  cu::SE3<float> T_mov_fix_;
  std::unique_ptr<ImageGpu32fC1> depth_proposal_;
  std::unique_ptr<ImageGpu32fC1> depth_proposal_sigma2_;

  // textures
  std::shared_ptr<Texture2D> lambda_tex_;
  std::shared_ptr<Texture2D> i1_tex_;
  std::shared_ptr<Texture2D> i2_tex_;
  std::shared_ptr<Texture2D> u_tex_;
  std::shared_ptr<Texture2D> u_prev_tex_;
  std::shared_ptr<Texture2D> u0_tex_;
  std::shared_ptr<Texture2D> pu_tex_;
  std::shared_ptr<Texture2D> q_tex_;
  std::shared_ptr<Texture2D> ix_tex_;
  std::shared_ptr<Texture2D> it_tex_;
  std::shared_ptr<Texture2D> xi_tex_;
  std::shared_ptr<Texture2D> g_tex_;

  std::shared_ptr<Texture2D> depth_proposal_tex_;
  std::shared_ptr<Texture2D> depth_proposal_sigma2_tex_;

//  std::shared_ptr<Texture2D> correspondence_guess_tex_;
//  std::shared_ptr<Texture2D> epi_vec_tex_;

};

} // namespace cu
} // namespace ze

#endif // IMP_CU_EPIPOLAR_STEREO_PRECOND_HUBER_L1_CUH
