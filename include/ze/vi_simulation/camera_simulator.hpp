#pragma once

#include <ze/cameras/camera_rig.h>
#include <ze/common/macros.h>
#include <ze/vi_simulation/trajectory_simulator.hpp>

namespace ze {

// fwd.
class Visualizer;

struct CameraSimulatorOptions
{
  uint32_t num_keypoints_per_frame { 50  };
  FloatType keypoint_noise_sigma { 1.0 };
  uint32_t max_num_landmarks_ { 10000 };
  FloatType min_depth { 2.0 };
  FloatType max_depth { 7.0 };
};

class CameraSimulator
{
public:
  ZE_POINTER_TYPEDEFS(CameraSimulator);

  CameraSimulator() = delete;

  CameraSimulator(
      const TrajectorySimulator::Ptr& trajectory,
      const CameraRig::Ptr& camera_rig,
      const CameraSimulatorOptions& options)
    : trajectory_(trajectory)
    , rig_(camera_rig)
    , options_(options)
  {}

  void setVisualizer(const std::shared_ptr<Visualizer>& visualizer);

  void initializeMap();

  void visualize(FloatType dt = 0.2, FloatType marker_size_trajectory = 0.2,
                 FloatType marker_size_landmarks = 0.2);

  size_t visibleLandmarks(
      const uint32_t cam_idx,
      const Transformation& T_W_B,
      const uint32_t lm_min_idx,
      const uint32_t lm_max_idx);

private:
  TrajectorySimulator::Ptr trajectory_;
  CameraRig::Ptr rig_;
  CameraSimulatorOptions options_;

  std::shared_ptr<Visualizer> viz_;

  Positions landmarks_W_;
  Bearings normals_W_;
};



} // namespace ze
