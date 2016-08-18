#include <ze/vi_simulation/camera_simulator.hpp>
#include <ze/cameras/camera_utils.h>
#include <ze/visualization/viz_interface.h>

namespace ze {

// -----------------------------------------------------------------------------
void CameraSimulator::initializeMap()
{
  uint32_t num_frames = options_.max_num_landmarks_
                         / options_.num_keypoints_per_frame;
  FloatType time = trajectory_->start();
  FloatType dt = (trajectory_->end() - time) / num_frames;
  int num_landmarks_per_frame = options_.num_keypoints_per_frame / rig_->size();
  uint32_t num_landmarks = 0u;
  landmarks_W_.resize(Eigen::NoChange, options_.max_num_landmarks_);

  for (uint32_t i = 0u; i < num_frames; ++i)
  {
    Transformation T_W_B = trajectory_->T_W_B(time);

    for (uint32_t cam_idx = 0u; cam_idx < rig_->size(); ++cam_idx)
    {
      // Check how many landmarks are visible:


      int num_visible = visibleLandmarks(cam_idx, T_W_B, 0u, num_landmarks);
      VLOG(1) << num_visible << " already visible.";

      if (num_visible >= num_landmarks_per_frame)
      {
        continue;
      }

      int32_t num_new_landmarks = std::max(0, num_landmarks_per_frame - num_visible);
      CHECK_GE(num_new_landmarks, 0);

      Positions p_C;
      std::tie(std::ignore, std::ignore, p_C) =
          generateRandomVisible3dPoints(
            rig_->at(cam_idx), num_new_landmarks,
            10u, options_.min_depth, options_.max_depth);

      DEBUG_CHECK_LE(static_cast<int>(num_landmarks + num_new_landmarks),
                     landmarks_W_.cols());

      landmarks_W_.middleCols(num_landmarks, num_new_landmarks)
          =  (T_W_B * rig_->T_B_C(cam_idx)).transformVectorized(p_C);

      num_landmarks += num_new_landmarks;
    }
    time += dt;
  }

  VLOG(1) << "Initialized map with " << num_landmarks << " visible landmarks.";
  landmarks_W_.conservativeResize(Eigen::NoChange, num_landmarks);
}

// -----------------------------------------------------------------------------
size_t CameraSimulator::visibleLandmarks(
    const uint32_t cam_idx,
    const Transformation& T_W_B,
    const uint32_t lm_min_idx,
    const uint32_t lm_max_idx)
{
  const uint32_t num_landmarks = lm_max_idx - lm_min_idx;
  if (num_landmarks == 0)
  {
    return 0;
  }

  const Size2u image_size = rig_->at(cam_idx).size();
  const auto lm_W = landmarks_W_.middleCols(lm_min_idx, num_landmarks);
  const auto lm_C = (T_W_B * rig_->T_B_C(cam_idx)).inverse().transformVectorized(lm_W);
  Keypoints px = rig_->at(cam_idx).projectVectorized(lm_C);
  std::vector<uint32_t> visible_indices;
  for (uint32_t i = 0u; i < num_landmarks; ++i)
  {
    if (lm_C(2,i) < options_.min_depth ||
        lm_C(2,i) > options_.max_depth)
    {
      // Landmark is either behind or too far from the camera.
      continue;
    }

    if (isVisible(image_size, px.col(i)))
    {
      visible_indices.push_back(i);
    }
  }

  return visible_indices.size();
}


// -----------------------------------------------------------------------------
void CameraSimulator::setVisualizer(const std::shared_ptr<Visualizer>& visualizer)
{
  viz_ = visualizer;
}

// -----------------------------------------------------------------------------
void CameraSimulator::visualize(
    FloatType dt,
    FloatType marker_size_trajectory,
    FloatType marker_size_landmarks)
{
  DEBUG_CHECK(viz_);
  std::vector<Position> trajectory;
  for (FloatType time = trajectory_->start(), time_end = trajectory_->end();
       time < time_end; time += dt)
  {
    trajectory.push_back(trajectory_->T_W_B(time).getPosition());
  }
  viz_->drawTrajectory("simulation_trajectory", 0, trajectory, Colors::Green, marker_size_trajectory);
  viz_->drawPoints("simulated_map", 0u, landmarks_W_, Colors::Orange, marker_size_landmarks);
}

} // namespace ze
