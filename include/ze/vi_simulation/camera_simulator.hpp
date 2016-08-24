// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <memory>
#include <unordered_map>
#include <ze/common/macros.h>
#include <ze/common/timer_collection.h>
#include <ze/common/transformation.h>
#include <ze/common/types.h>
#include <ze/vi_simulation/camera_simulator_types.hpp>

namespace ze {

// fwd.
class CameraRig;
class TrajectorySimulator;
class Visualizer;

// -----------------------------------------------------------------------------
struct CameraSimulatorOptions
{
  uint32_t num_keypoints_per_frame { 50  };
  real_t keypoint_noise_sigma { 1.0 };
  uint32_t max_num_landmarks_ { 10000 };
  real_t min_depth_m { 2.0 };
  real_t max_depth_m { 7.0 };
};

// -----------------------------------------------------------------------------
//! Simulate feature observations while moving along a trajectory.
//! @todo(cfo) Model extra nuisances:
//!            - false associations / outliers
//!            - variable number of features along trajectory
class CameraSimulator
{
public:
  ZE_POINTER_TYPEDEFS(CameraSimulator);

  CameraSimulator() = delete;

  CameraSimulator(
      const std::shared_ptr<TrajectorySimulator>& trajectory,
      const std::shared_ptr<CameraRig>& camera_rig,
      const CameraSimulatorOptions& options)
    : trajectory_(trajectory)
    , rig_(camera_rig)
    , options_(options)
  {}

  void setVisualizer(const std::shared_ptr<Visualizer>& visualizer);

  void initializeMap();

  void visualize(
      real_t dt = 0.2,
      real_t marker_size_trajectory = 0.2,
      real_t marker_size_landmarks = 0.2);

  CameraMeasurementsVector getMeasurements(
      real_t time);

  CameraMeasurementsVector getMeasurementsCorrupted(
      real_t time);

  void reset();

  inline const TrajectorySimulator& trajectory() const { return *trajectory_; }

  DECLARE_TIMER(SimTimer, timer_,
                visible_landmarks, get_measurements);

private:
  CameraMeasurements visibleLandmarks(
      const uint32_t cam_idx,
      const Transformation& T_W_B,
      const uint32_t lm_min_idx,
      const uint32_t lm_max_idx);

  std::shared_ptr<TrajectorySimulator> trajectory_;
  std::shared_ptr<CameraRig> rig_;
  CameraSimulatorOptions options_;

  std::shared_ptr<Visualizer> viz_;

  Positions landmarks_W_;
  Bearings normals_W_;

  int32_t track_id_counter_ = 0;
  std::unordered_map<int32_t, int32_t> global_lm_id_to_track_id_map_;
};



} // namespace ze
