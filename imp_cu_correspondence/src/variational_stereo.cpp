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
#include <imp/cu_correspondence/variational_stereo.hpp>
#include <imp/cu_correspondence/stereo_ctf_warping.hpp>

namespace ze {
namespace cu {

//------------------------------------------------------------------------------
VariationalStereo::VariationalStereo(Parameters::Ptr params)
{
  if (params)
  {
    params_ = params;
  }
  else
  {
    params_ = std::make_shared<Parameters>();
  }

  ctf_.reset(new StereoCtFWarping(params_));
}

//------------------------------------------------------------------------------
VariationalStereo::~VariationalStereo()
{ ; }


//------------------------------------------------------------------------------
void VariationalStereo::addImage(const ImageGpu32fC1::Ptr& image)
{
  CHECK(image != nullptr) << "Invalid input image.";
  ctf_->addImage(image);
}

//------------------------------------------------------------------------------
void VariationalStereo::reset()
{
  ctf_->reset();
}

//------------------------------------------------------------------------------
void VariationalStereo::solve()
{
  ctf_->solve();
}

//------------------------------------------------------------------------------
ImageGpu32fC1::Ptr VariationalStereo::computePrimalEnergy(size_t level)
{
  CHECK_LT(level, params_->ctf.coarsest_level);
  return ctf_->computePrimalEnergy(level);
}

//------------------------------------------------------------------------------
VariationalStereo::ImageGpu32fC1::Ptr VariationalStereo::getDisparities(size_t level)
{
  CHECK_LT(level, params_->ctf.coarsest_level);
  return ctf_->getDisparities(level);
}


//------------------------------------------------------------------------------
VariationalStereo::ImageGpu32fC1::Ptr VariationalStereo::getOcclusion(size_t level)
{
  CHECK_LT(level, params_->ctf.coarsest_level);
  return ctf_->getOcclusion(level);
}


} // namespace cu
} // namespace ze

