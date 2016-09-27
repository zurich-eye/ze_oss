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

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_correspondence/variational_stereo_parameters.hpp>

namespace ze {
namespace cu {

// forward declarations
class StereoCtFWarping;

/**
 * @brief The Stereo class takes a stereo image pair and estimates the disparity map
 */
class VariationalStereo
{
public:
  //! @todo (MWE) first do the implementation with specific type (32fC1) and later generalize
  using ImageGpu32fC1 = ze::cu::ImageGpu32fC1;
  using Parameters = VariationalStereoParameters;

public:
  VariationalStereo(Parameters::Ptr params=nullptr);
  virtual ~VariationalStereo(); //= default;

  virtual void addImage(const ImageGpu32fC1::Ptr& image);
  virtual void reset();
  virtual void solve();

  virtual ImageGpu32fC1::Ptr computePrimalEnergy(size_t level=0);
  virtual ImageGpu32fC1::Ptr getDisparities(size_t level=0);
  virtual ImageGpu32fC1::Ptr getOcclusion(size_t level=0);

  // getters / setters
  virtual inline Parameters::Ptr parameters() {return params_;}

protected:
  Parameters::Ptr params_;  //!< configuration parameters
  std::unique_ptr<StereoCtFWarping> ctf_;  //!< performing a coarse-to-fine warping scheme
};

} // namespace cu
} // namespace ze
