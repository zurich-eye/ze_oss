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

#include <ze/common/types.hpp>
#include <ze/common/macros.hpp>
#include <imp/core/types.hpp>
#include <imp/cu_correspondence/stereo_solver_enum.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace ze {
namespace cu {

// the parameter struct
struct VariationalStereoParameters
{
  ZE_POINTER_TYPEDEFS(VariationalStereoParameters);

  StereoPDSolver solver=StereoPDSolver::PrecondHuberL1; //!< selected primal-dual solver / model combination
  float lambda = 30.0f; //!< tradeoff between regularization and matching term (R(u) + \lambda * D(u))
  ImageGpu32fC1::Ptr lambda_pointwise = nullptr; //!< pointwise variant of lambda
  float eps_u = 0.05f; //!< tradeoff between L1 and L2 part of the Huber regularization

  float edge_sigma = 1.f;
  float edge_alpha = 7.f;
  float edge_q = 0.7f;

  //! @todo (MWE) we might want to define this externally for all ctf approaches?
  // settings for the ctf warping
  struct CTF
  {
    float scale_factor = 0.8f; //!< multiplicative scale factor between coarse-to-fine pyramid levels
    uint32_t iters = 100;
    uint32_t warps =  10;
    size_t levels = UINT32_MAX;
    size_t coarsest_level = UINT32_MAX;
    size_t finest_level = 0;
    bool apply_median_filter = true;
  } ctf;

  friend std::ostream& operator<<(std::ostream& stream,
                                  const VariationalStereoParameters& p);
};

} // namespace cu
} // namespace ze
