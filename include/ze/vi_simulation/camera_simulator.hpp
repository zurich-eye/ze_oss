#pragma once

#include <unordered_map>
#include <ze/cameras/camera_rig.h>
#include <ze/common/macros.h>
#include <ze/vi_simulation/trajectory_simulator.hpp>
#include <ze/common/timer_collection.h>

namespace ze {

// fwd.
class Visualizer;

// -----------------------------------------------------------------------------
struct CameraMeasurements
{
  //! Each column is a keypoint observation.
  Keypoints keypoints_;

  //! Global landmark index of each observed feature. The size of the vector is
  //! the same as the number of columns in the keypoints block.
  std::vector<int32_t> global_landmark_ids_;

  //! Temporary track index of a landmark. If the landmark is
  //! re-observed after a loop, it will be assigned a different id.
  std::vector<int32_t> local_track_ids_;
};
using CameraMeasurementsVector = std::vector<CameraMeasurements>;

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
      const TrajectorySimulator::Ptr& trajectory,
      const CameraRig::Ptr& camera_rig,
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

  TrajectorySimulator::Ptr trajectory_;
  CameraRig::Ptr rig_;
  CameraSimulatorOptions options_;

  std::shared_ptr<Visualizer> viz_;

  Positions landmarks_W_;
  Bearings normals_W_;

  int32_t track_id_counter_ = 0;
  std::unordered_map<int32_t, int32_t> global_lm_id_to_track_id_map_;
};



} // namespace ze
