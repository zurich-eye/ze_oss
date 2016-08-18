#include <ze/vi_simulation/camera_simulator.hpp>
#include <ze/cameras/camera_utils.h>

namespace ze {

void CameraSimulator::initializeMap()
{
  uint32_t num_frames = options_.max_num_landmarks_
                         / options_.num_keypoints_per_frame;
  FloatType time = trajectory_->start();
  FloatType dt = (trajectory_->end() - time) / num_frames;
  uint32_t num_landmarks_per_frame = options_.num_keypoints_per_frame / rig_->size();
  uint32_t num_landmarks = 0u;

  for (uint32_t i = 0u; i < num_frames; ++i)
  {
    Transformation T_W_B = trajectory_->T_W_B(time);

    for (uint32_t cam_idx = 0u; cam_idx < rig_->size(); ++cam_idx)
    {
      Positions p_C;
      std::tie(std::ignore, std::ignore, p_C) =
          generateRandomVisible3dPoints(rig_->at(cam_idx), num_landmarks_per_frame,
                                        10u, options_.min_depth, options_.max_depth);

      DEBUG_CHECK_LT(static_cast<int>(num_landmarks + num_landmarks_per_frame),
                     landmarks_W_.cols());

      landmarks_W_.middleCols(num_landmarks, num_landmarks_per_frame)
          =  (T_W_B * rig_->T_B_C(cam_idx)).transformVectorized(p_C);

      num_landmarks += num_landmarks_per_frame;
    }
    time += dt;
  }

  VLOG(1) << "Initialized map with " << num_landmarks << " visible landmarks.";
  landmarks_W_.conservativeResize(Eigen::NoChange, num_landmarks);
}

} // namespace ze
