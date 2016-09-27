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

#include <cstdint>
#include <memory>

#include <imp/cu_core/cu_matrix.cuh>
#include <imp/cu_core/cu_se3.cuh>
#include <imp/cu_core/cu_pinhole_camera.cuh>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_correspondence/variational_stereo_parameters.hpp>
#include <imp/cu_correspondence/variational_stereo.hpp>

namespace ze {
namespace cu {

/**
 * @brief The Stereo class takes an image pair with known epipolar geometry
 *        (fundamental matrix) and estimates the disparity map
 */
class VariationalEpipolarStereo : public VariationalStereo
{
public:
  using VectorImage = ze::cu::ImageGpu32fC2;
  using Cameras = std::vector<cu::PinholeCamera>;

public:
  VariationalEpipolarStereo(Parameters::Ptr params=nullptr);
  virtual ~VariationalEpipolarStereo(); //= default;

  virtual void setFundamentalMatrix(const cu::Matrix3f& F);
  virtual void setIntrinsics(const Cameras& cams);
  virtual void setExtrinsics(const cu::SE3<float>& T_mov_fix);
  virtual void setDepthProposal(
      const ImageGpu32fC1::Ptr& depth_proposal,
      const ImageGpu32fC1::Ptr& depth_proposal_sigma2=nullptr);

private:
};

} // namespace cu
} // namespace ze

