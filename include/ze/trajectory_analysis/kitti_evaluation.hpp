// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

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
