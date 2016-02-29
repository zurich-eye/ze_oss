#pragma once

#include <ze/common/transformation.h>

namespace ze {

struct RelativeError
{
  size_t first_frame;
  Vector3 W_t_gt_es;  //!< Relative translation error represented in world frame.
  Vector3 W_R_gt_es;  //!< Relative rotation error (Angle-Axis) in world frame.
  FloatType len;
  FloatType scale_error;
  int num_frames;

  RelativeError(
      size_t first_frame, Vector3 W_t_gt_es, Vector3 W_R_gt_es,
      FloatType segment_length, FloatType scale_error,
      int num_frames_in_between);
};

std::vector<FloatType> trajectoryDistances(
    const TransformationVector& poses);

int32_t lastFrameFromSegmentLength(
    const std::vector<FloatType>& dist,
    const size_t first_frame,
    const FloatType segment_length);

std::vector<RelativeError> calcSequenceErrors(
    const TransformationVector& poses_gt,
    const TransformationVector& poses_es,
    const FloatType& segment_length,
    const size_t skip_num_frames_between_segment_evaluation,
    const bool use_least_squares_alignment,
    const double least_squares_align_range);

} // namespace ze
