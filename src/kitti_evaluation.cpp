#include <ze/trajectory_analysis/kitti_evaluation.h>

#include <ze/geometry/align_poses.h>

namespace ze {

RelativeError::RelativeError(
    size_t first_frame, Vector3 W_t_gt_es, Vector3 W_R_gt_es,
    FloatType segment_length, int num_frames_in_between)
  : first_frame(first_frame)
  , W_t_gt_es(W_t_gt_es)
  , W_R_gt_es(W_R_gt_es)
  , len(segment_length)
  , num_frames(num_frames_in_between)
{}

std::vector<FloatType> trajectoryDistances(
    const TransformationVector& poses)
{
  std::vector<FloatType> dist;
  dist.reserve(poses.size());
  dist.push_back(0);
  for (size_t i = 1; i < poses.size(); ++i)
  {
    dist.push_back(dist.at(i-1) + (poses.at(i).getPosition() - poses.at(i-1).getPosition()).norm());
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
    if (dist.at(i) > dist.at(first_frame) + segment_length)
      return i;
  }
  return -1;
}

std::vector<RelativeError> calcSequenceErrors(
    const TransformationVector& T_W_gt,
    const TransformationVector& T_W_es,
    const FloatType& segment_length,
    const size_t skip_num_frames_between_segment_evaluation,
    const bool use_least_squares_alignment)
{
  // Pre-compute cumulative distances (from ground truth as reference).
  std::vector<FloatType> dist = trajectoryDistances(T_W_gt);

  // Compute relative errors for all start positions.
  std::vector<RelativeError> errors;
  for (size_t first_frame = 0; first_frame < T_W_gt.size();
       first_frame += skip_num_frames_between_segment_evaluation)
  {
    // Find last frame to compare with.
    int32_t last_frame = lastFrameFromSegmentLength(dist, first_frame, segment_length);
    if (last_frame == -1)
    {
      continue; // continue, if segment is longer than trajectory.
    }

    // Perform a least-squares alignment of the first 20% of the trajectories.
    int n_align_poses = 0.2 * (last_frame - first_frame);
    Transformation T_gt_es = T_W_gt[first_frame].inverse() * T_W_es[first_frame];
    if(use_least_squares_alignment && n_align_poses > 1)
    {
      TransformationVector T_W_es_align(
            T_W_es.begin() + first_frame, T_W_es.begin() + first_frame + n_align_poses);
      TransformationVector T_W_gt_align(
            T_W_gt.begin() + first_frame, T_W_gt.begin() + first_frame + n_align_poses);
      VLOG(40) << "T_W_es_align size = " << T_W_es_align.size();

      const FloatType sigma_pos = 0.05;
      const FloatType sigma_rot = 5.0 / 180 * M_PI;
      PoseAligner problem(T_W_gt_align, T_W_es_align, sigma_pos, sigma_rot);

      Transformation T_gt_es_optimized = T_gt_es;
      problem.optimize(T_gt_es);
      Transformation T_diff = T_gt_es * T_gt_es_optimized.inverse();
      std::cout << T_diff.log().transpose() << std::endl;
    }

    // Compute relative rotational and translational errors.
    Transformation T_W_gtlast = T_W_gt[last_frame];
    Transformation T_W_eslast = T_W_es[last_frame];
    Transformation T_gtfirst_gtlast = T_W_gt[first_frame].inverse() * T_W_gtlast;
    //Transformation T_esfirst_eslast = T_W_es[first_frame].inverse() * T_W_eslast;
    //Transformation T_esfirst_eslast = (T_W_es[first_frame] * T_gt_es.inverse()).inverse() * (T_W_eslast * T_gt_es.inverse());
    Transformation T_esfirst_eslast = T_W_es[first_frame].inverse() * T_W_eslast;
    Transformation T_gtlast_eslast = T_gtfirst_gtlast.inverse() * T_esfirst_eslast;

    // The relative error is represented in the frame of reference of the last
    // frame in the ground-truth trajectory. We want to express it in the world
    // frame to make statements about the yaw drift (not observable in Visual-
    // inertial setting) vs. roll and pitch (observable).
    Vector3 W_t_gtlast_eslast = T_W_gtlast.getRotation().rotate(T_gtlast_eslast.getPosition());
    Vector3 W_R_gtlast_eslast = T_W_gtlast.getRotation().rotate(T_gtlast_eslast.getRotation().log());

    errors.push_back(RelativeError(first_frame,
                                   W_t_gtlast_eslast,
                                   W_R_gtlast_eslast,
                                   segment_length,
                                   last_frame - first_frame + 1));
  }

  // return error vector
  return errors;
}

} // namespace ze
