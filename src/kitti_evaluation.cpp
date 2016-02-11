#include <ze/trajectory_analysis/kitti_evaluation.h>

namespace ze {

RelativeError::RelativeError(
    size_t first_frame, FloatType r_err, FloatType t_err, FloatType segment_length,
    int num_frames_in_between)
  : first_frame(first_frame)
  , rot_error(r_err)
  , tran_error(t_err)
  , len(segment_length)
  , num_frames(num_frames_in_between)
{}

std::vector<FloatType> trajectoryDistances(
    const TransformationVector& poses)
{
  std::vector<FloatType> dist;
  dist.reserve(poses.size());
  dist.push_back(0);
  for (size_t i = 0; i < poses.size(); ++i)
  {
    dist.push_back(dist[i-1] + (poses[i].getPosition() - poses[i-1].getPosition()).norm());
  }
  return dist;
}

int32_t lastFrameFromSegmentLength(
    const std::vector<FloatType>& dist,
    const size_t first_frame,
    const FloatType segment_length)
{
  for (size_t i = first_frame; i < dist.size(); i++)
  {
    if (dist[i] > dist[first_frame] + segment_length)
      return i;
  }
  return -1;
}

std::vector<RelativeError> calcSequenceErrors(
    const TransformationVector& poses_gt,
    const TransformationVector& poses_es,
    const std::vector<FloatType>& segment_lengths,
    const size_t skip_num_frames_between_segment_evaluation)
{
  // error vector
  std::vector<RelativeError> errors;

  // pre-compute distances (from ground truth as reference)
  std::vector<FloatType> dist = trajectoryDistances(poses_gt);

  // for all start positions do
  for (size_t first_frame = 0; first_frame < poses_gt.size();
       first_frame += skip_num_frames_between_segment_evaluation)
  {
    // for all segment lengths do
    for (const FloatType len : segment_lengths)
    {

      // compute last frame
      int32_t last_frame = lastFrameFromSegmentLength(dist, first_frame, len);

      // continue, if sequence not long enough
      if(last_frame == -1)
      {
        continue;
      }

      // compute rotational and translational errors
      Transformation rel_pose_gt = poses_gt[first_frame].inverse() * poses_gt[last_frame];
      Transformation rel_pose_es = poses_es[first_frame].inverse() * poses_es[last_frame];
      Transformation rel_pose_error = rel_pose_es.inverse() * rel_pose_gt;
      FloatType rot_err = rel_pose_error.getRotation().log().norm();
      FloatType pos_err = rel_pose_error.getPosition().norm();

      // write to file
      errors.push_back(RelativeError(first_frame, rot_err/len, pos_err/len, len,
                                     last_frame - first_frame));
    }
  }

  // return error vector
  return errors;
}

} // namespace ze
