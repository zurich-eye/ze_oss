#pragma once

#include <ze/common/transformation.h>

namespace ze {

struct RelativeError
{
  size_t first_frame;
  FloatType rot_error;
  FloatType tran_error;
  FloatType len;
  int num_frames;

  RelativeError(
      size_t first_frame, FloatType r_err, FloatType t_err,
      FloatType segment_length, int num_frames_in_between);
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
    const size_t skip_num_frames_between_segment_evaluation);

} // namespace ze
