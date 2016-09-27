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
#include <imp/cu_correspondence/variational_epipolar_stereo.hpp>

#include <imp/cu_correspondence/stereo_ctf_warping.hpp>

namespace ze {
namespace cu {

//------------------------------------------------------------------------------
VariationalEpipolarStereo::VariationalEpipolarStereo(Parameters::Ptr params)
  : VariationalStereo(params)
{ ; }

//------------------------------------------------------------------------------
VariationalEpipolarStereo::~VariationalEpipolarStereo()
{ ; }

//------------------------------------------------------------------------------
void VariationalEpipolarStereo::setFundamentalMatrix(const cu::Matrix3f& F)
{
  ctf_->setFundamentalMatrix(F);
}


//------------------------------------------------------------------------------
void VariationalEpipolarStereo::setIntrinsics(const Cameras& cams)
{
  ctf_->setIntrinsics(cams);
}

//------------------------------------------------------------------------------
void VariationalEpipolarStereo::setExtrinsics(const cu::SE3<float>& T_mov_fix)
{
  ctf_->setExtrinsics(T_mov_fix);
}

//------------------------------------------------------------------------------
void VariationalEpipolarStereo::setDepthProposal(
    const ImageGpu32fC1::Ptr& depth_proposal,
    const ImageGpu32fC1::Ptr& depth_proposal_sigma2)
{
  ctf_->setDepthProposal(depth_proposal, depth_proposal_sigma2);
}


} // namespace cu
} // namespace ze

