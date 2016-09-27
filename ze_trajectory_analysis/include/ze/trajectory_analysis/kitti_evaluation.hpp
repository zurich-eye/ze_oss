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

#include <ze/common/transformation.hpp>

namespace ze {

struct RelativeError
{
  size_t first_frame;
  Vector3 W_t_gt_es;  //!< Relative translation error represented in world frame.
  Vector3 W_R_gt_es;  //!< Relative rotation error (Angle-Axis) in world frame.
  real_t len;
  real_t scale_error;
  int num_frames;

  RelativeError(
      size_t first_frame, Vector3 W_t_gt_es, Vector3 W_R_gt_es,
      real_t segment_length, real_t scale_error,
      int num_frames_in_between);
};

std::vector<real_t> trajectoryDistances(
    const TransformationVector& poses);

int32_t lastFrameFromSegmentLength(
    const std::vector<real_t>& dist,
    const size_t first_frame,
    const real_t segment_length);

std::vector<RelativeError> calcSequenceErrors(
    const TransformationVector& poses_gt,
    const TransformationVector& poses_es,
    const real_t& segment_length,
    const size_t skip_num_frames_between_segment_evaluation,
    const bool use_least_squares_alignment,
    const double least_squares_align_range,
    const bool least_squares_align_translation_only);

} // namespace ze
