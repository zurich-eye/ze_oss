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
#include <imp/core/size.hpp>
#include <imp/cu_correspondence/variational_stereo_parameters.hpp>


namespace ze {
namespace cu {

/**
 * @brief The StereoCtFWarpingLevel class
 */
class SolverStereoAbstract
{
public:
  using ImageGpu32fC1 = ze::cu::ImageGpu32fC1;
  using ImageGpu32fC2 = ze::cu::ImageGpu32fC2;
  using Parameters = VariationalStereoParameters;

public:
  SolverStereoAbstract() = delete;
  virtual ~SolverStereoAbstract() = default;

  SolverStereoAbstract(Parameters::Ptr params,
                       ze::Size2u size, std::uint16_t level)
    : params_(params)
    , size_(size)
    , level_(level)
  { ; }

  virtual void init() = 0;
  virtual void init(const SolverStereoAbstract& rhs) = 0;
  virtual void solve(std::vector<ImageGpu32fC1::Ptr> images) = 0;

  /**
   * @brief computePrimalEnergy returns an the primal energy with the current disparity values.
   * @note There is no need to implement this function so by default a nullptr is returned
   * @return Pixel-wise primal energy
   */
  virtual ImageGpu32fC1::Ptr computePrimalEnergy() {return nullptr;}

  virtual ImageGpu32fC1::Ptr getDisparities() = 0;

  /**
   * @brief getOcclusion returns an estimate of occluded pixels
   * @note There is no need to implement this function so by default a nullptr is returned
   * @return A mask with an estimate of occluded pixels or nullptr if not estimated.
   */
  virtual ImageGpu32fC1::Ptr getOcclusion() {return nullptr;}


  // setters / getters
  inline ze::Size2u size() { return size_; }
  inline std::uint16_t level() { return level_; }

protected:
  Parameters::Ptr params_; //!< configuration parameters
  ze::Size2u size_;
  std::uint16_t level_; //!< level number in the ctf pyramid (0=finest .. n=coarsest)
};

} // namespace cu
} // namespace ze

