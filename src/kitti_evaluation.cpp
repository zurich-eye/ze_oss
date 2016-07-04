#include <ze/trajectory_analysis/kitti_evaluation.h>

#include <ze/geometry/align_poses.h>
#include <ze/geometry/align_points.h>

namespace ze {

RelativeError::RelativeError(
    size_t first_frame, Vector3 W_t_gt_es, Vector3 W_R_gt_es,
    FloatType segment_length, FloatType scale_error, int num_frames_in_between)
  : first_frame(first_frame)
  , W_t_gt_es(W_t_gt_es)
  , W_R_gt_es(W_R_gt_es)
  , len(segment_length)
  , scale_error(scale_error)
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
    const TransformationVector& T_W_A, // groundtruth
    const TransformationVector& T_W_B,
    const FloatType& segment_length,
    const size_t skip_num_frames_between_segment_evaluation,
    const bool use_least_squares_alignment,
    const double least_squares_align_range,
    const bool least_squares_align_translation_only)
{
  // Pre-compute cumulative distances (from ground truth as reference).
  std::vector<FloatType> dist_gt = trajectoryDistances(T_W_A);
  std::vector<FloatType> dist_es = trajectoryDistances(T_W_B);

  // Compute relative errors for all start positions.
  std::vector<RelativeError> errors;
  for (size_t first_frame = 0; first_frame < T_W_A.size();
       first_frame += skip_num_frames_between_segment_evaluation)
  {
    // Find last frame to compare with.
    int32_t last_frame = lastFrameFromSegmentLength(dist_gt, first_frame, segment_length);
    if (last_frame == -1)
    {
      continue; // continue, if segment is longer than trajectory.
    }

    // Perform a least-squares alignment of the first part of the trajectories.
    int n_align_poses = least_squares_align_range * (last_frame - first_frame);
    Transformation T_A0_B0 = T_W_A[first_frame].inverse() * T_W_B[first_frame];
    if(use_least_squares_alignment && n_align_poses > 1)
    {
      if (least_squares_align_translation_only)
      {
        /*
        Positions p_W_es_align(3, n_align_poses);
        Positions p_W_gt_align(3, n_align_poses);
        for (int i = 0; i < n_align_poses; ++i)
        {
          p_W_es_align.col(i) = T_W_B.at(first_frame + i).getPosition();
          p_W_gt_align.col(i) = T_W_A.at(first_frame + i).getPosition();

          PointAligner problem(p_W_gt_align, p_W_es_align);
          problem.optimize(T_A0_B0);
        }
        */
        //! @todo: Run closed-form solution of the alignment first. As it is now
        //!        the alignment does not provide good results.
        LOG(FATAL) << "Point-based alignment not supported yet.";
      }
      else
      {
        TransformationVector T_W_es_align(
              T_W_B.begin() + first_frame, T_W_B.begin() + first_frame + n_align_poses);
        TransformationVector T_W_gt_align(
              T_W_A.begin() + first_frame, T_W_A.begin() + first_frame + n_align_poses);
        VLOG(40) << "T_W_es_align size = " << T_W_es_align.size();

        const FloatType sigma_pos = 0.05;
        const FloatType sigma_rot = 5.0 / 180 * M_PI;
        PoseAligner problem(T_W_gt_align, T_W_es_align, sigma_pos, sigma_rot);
        problem.optimize(T_A0_B0);
      }
    }

    // Compute relative rotational and translational errors.
    Transformation T_W_Ai = T_W_A[last_frame];
    Transformation T_A0_Ai = T_W_A[first_frame].inverse() * T_W_Ai;
    Transformation T_Bi_A0 = T_W_B[last_frame].inverse() * T_W_A[first_frame];
    Transformation T_Bi_Ai = T_Bi_A0 * T_A0_B0 * T_A0_Ai;
    Transformation T_Ai_Bi = T_Bi_Ai.inverse();

    // The relative error is represented in the frame of reference of the last
    // frame in the ground-truth trajectory. We want to express it in the world
    // frame to make statements about the yaw drift (not observable in Visual-
    // inertial setting) vs. roll and pitch (observable).
    Vector3 W_t_gtlast_eslast = T_W_Ai.getRotation().rotate(T_Ai_Bi.getPosition());
    Vector3 W_R_gtlast_eslast = T_W_Ai.getRotation().rotate(T_Ai_Bi.getRotation().log());

    // Scale error is the ratio of the respective segment length
    FloatType scale_error =
        (dist_es.at(last_frame) - dist_es.at(first_frame))
        / (dist_gt.at(last_frame) - dist_gt.at(first_frame));

    errors.push_back(RelativeError(first_frame,
                                   W_t_gtlast_eslast,
                                   W_R_gtlast_eslast,
                                   segment_length,
                                   scale_error,
                                   last_frame - first_frame + 1));
  }

  // return error vector
  return errors;
}

} // namespace ze
