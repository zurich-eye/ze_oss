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
#pragma once

#include <memory>
#include <vector>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_imgproc/image_pyramid.hpp>
#include <imp/cu_core/cu_pinhole_camera.cuh>
#include <imp/cu_core/cu_se3.cuh>
#include <imp/cu_core/cu_matrix.cuh>

#include <imp/cu_correspondence/variational_stereo_parameters.hpp>


namespace ze {
namespace cu {

// forward declarations
class SolverStereoAbstract;

/**
 * @brief The StereoCtFWarping class
 * @todo (MWE) better handling of fixed vs. moving images when adding (incremental updates)
 * @todo (MWE) better interface for multiple input images with fundamental matrix prior
 */
class StereoCtFWarping
{
public:
  using Parameters = VariationalStereoParameters;

  using ImageGpu32fC1 = ze::cu::ImageGpu32fC1;
  using ImageGpu32fC2 = ze::cu::ImageGpu32fC2;
  using ImagePyramid32fC1 = ze::ImagePyramid32fC1;

  using Cameras = std::vector<cu::PinholeCamera>;
  using CamerasPyramid = std::vector<Cameras>;

public:
  StereoCtFWarping() = delete;
  virtual ~StereoCtFWarping();

  StereoCtFWarping(Parameters::Ptr params);
  StereoCtFWarping(ze::Size2u image_size, uint8_t num_images,
                   Parameters::Ptr params);

  void addImage(const ImageGpu32fC1::Ptr& image);
  void reset();
  void solve();
  ImageGpu32fC1::Ptr computePrimalEnergy(size_t level=0);
  ImageGpu32fC1::Ptr getDisparities(size_t level=0);
  ImageGpu32fC1::Ptr getOcclusion(size_t level=0);

  // if we have a guess about the correspondence points and the epipolar geometry
  // given we can set these as a prior
  inline virtual void setFundamentalMatrix(const cu::Matrix3f& F) {F_ = F;}
  virtual void setIntrinsics(const Cameras& cams) {cams_ = cams;}
  virtual void setExtrinsics(const cu::SE3<float>& T_mov_fix) {T_mov_fix_=T_mov_fix;}

  inline virtual void setDepthProposal(
      const ImageGpu32fC1::Ptr& depth_proposal,
      const ImageGpu32fC1::Ptr& depth_proposal_sigma2=nullptr)
  {
    depth_proposal_ = depth_proposal;
    depth_proposal_sigma2_ = depth_proposal_sigma2;
  }

protected:
  /**
   * @brief ready checks if everything is setup and initialized.
   * @return State if everything is ready to solve the given problem.
   */
  bool ready();

  /**
   * @brief init initializes the solvers for the current setup
   */
  void init();

private:
  Parameters::Ptr params_; //!< configuration parameters
  std::vector<ImageGpu32fC1::Ptr> images_; //!< all unprocessed input images
  std::vector<ImagePyramid32fC1::Ptr> image_pyramids_; //!< image pyramids corresponding to the unprocesed input images
  std::vector<std::unique_ptr<SolverStereoAbstract>> levels_;

  cu::Matrix3f F_;
  std::vector<cu::PinholeCamera> cams_;
  cu::SE3<float> T_mov_fix_;

  ImageGpu32fC1::Ptr depth_proposal_;
  ImageGpu32fC1::Ptr depth_proposal_sigma2_;
};

} // namespace cu
} // namespace ze
