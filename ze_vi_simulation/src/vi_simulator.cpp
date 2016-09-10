// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/vi_simulation/vi_simulator.hpp>

#include <ze/cameras/camera_rig.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/csv_trajectory.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/vi_simulation/trajectory_simulator.hpp>
#include <ze/vi_simulation/camera_simulator.hpp>
#include <ze/vi_simulation/imu_simulator.hpp>
#include <ze/visualization/viz_interface.hpp>

namespace ze {

// -----------------------------------------------------------------------------
ViSimulator::ViSimulator(
    const TrajectorySimulator::Ptr& trajectory,
    const CameraRig::Ptr& camera_rig,
    const CameraSimulatorOptions& camera_sim_options,
    const real_t gyr_bias_noise_sigma,
    const real_t acc_bias_noise_sigma,
    const real_t gyr_noise_sigma,
    const real_t acc_noise_sigma,
    const uint32_t cam_framerate_hz,
    const uint32_t imu_bandwidth_hz,
    const real_t gravity_magnitude)
  : trajectory_(trajectory)
  , cam_dt_ns_(secToNanosec(1.0 / cam_framerate_hz))
  , imu_dt_ns_(secToNanosec(1.0 / imu_bandwidth_hz))
  , last_sample_stamp_ns_(secToNanosec(trajectory->start()))
{
  CHECK(imu_bandwidth_hz % cam_framerate_hz == 0);

  ImuBiasSimulator::Ptr bias;
  try
  {
    VLOG(1) << "Initialize bias ...";
    bias = std::make_shared<ContinuousBiasSimulator>(
             Vector3::Constant(gyr_bias_noise_sigma),
             Vector3::Constant(acc_bias_noise_sigma),
             trajectory->start(),
             trajectory->end(),
             100); // Results in malloc: (trajectory->end() - trajectory->start()) * imu_bandwidth_hz);
    VLOG(1) << "done.";
  }
  catch (const std::bad_alloc& e)
  {
    LOG(FATAL) << "Could not create bias because number of samples is too high."
               << " Allocation failed: " << e.what();
  }

  VLOG(1) << "Initialize IMU ...";
  imu_ = std::make_shared<ImuSimulator>(
           trajectory,
           bias,
           RandomVectorSampler<3>::sigmas(Vector3::Constant(acc_noise_sigma)),
           RandomVectorSampler<3>::sigmas(Vector3::Constant(gyr_noise_sigma)),
           imu_bandwidth_hz,
           imu_bandwidth_hz,
           gravity_magnitude);
  VLOG(1) << "done.";

  camera_ = std::make_shared<CameraSimulator>(
              trajectory,
              camera_rig,
              camera_sim_options);
}

// -----------------------------------------------------------------------------
void ViSimulator::initialize()
{
  VLOG(1) << "Initialize map ...";
  camera_->initializeMap();
  VLOG(1) << "done.";
}

// -----------------------------------------------------------------------------
std::pair<ViSensorData, bool> ViSimulator::getMeasurement()
{
  int64_t new_img_stamp_ns = last_sample_stamp_ns_ + cam_dt_ns_;

  real_t time_s = nanosecToSecTrunc(new_img_stamp_ns);
  ViSensorData data;

  if (time_s > trajectory_->end())
  {
    LOG(WARNING) << "Reached end of trajectory!";
    return std::make_pair(data, false);
  }

  data.timestamp = new_img_stamp_ns;

  // Get camera measurements:
  data.cam_measurements = camera_->getMeasurementsCorrupted(time_s);

  // Get groundtruth:
  data.groundtruth.T_W_Bk = trajectory_->T_W_B(time_s);
  data.groundtruth.linear_velocity_W = trajectory_->velocity_W(time_s);
  data.groundtruth.angular_velocity_B = trajectory_->angularVelocity_B(time_s);
  data.groundtruth.acc_bias = imu_->bias()->accelerometer(time_s);
  data.groundtruth.gyr_bias = imu_->bias()->gyroscope(time_s);
  T_W_Bk_ = data.groundtruth.T_W_Bk;

  // Get inertial measurements:
  uint64_t imu_stamp_ns = last_sample_stamp_ns_;
  uint32_t num_imu_measurements = cam_dt_ns_ / imu_dt_ns_ + 1u;

  data.imu_measurements.resize(Eigen::NoChange, num_imu_measurements);
  data.imu_stamps.resize(num_imu_measurements);
  for (uint32_t i = 0; i < num_imu_measurements; ++i)
  {
    data.imu_stamps(i) = imu_stamp_ns;
    data.imu_measurements.block<3,1>(0, i) =
        imu_->specificForceCorrupted(nanosecToSecTrunc(imu_stamp_ns));
    data.imu_measurements.block<3,1>(3, i) =
        imu_->angularVelocityCorrupted(nanosecToSecTrunc(imu_stamp_ns));
    imu_stamp_ns += imu_dt_ns_;
  }
  DEBUG_CHECK_EQ(data.imu_stamps(data.imu_stamps.size()-1), new_img_stamp_ns);

  // Prepare next iteration:
  last_sample_stamp_ns_ = new_img_stamp_ns;

  return std::make_pair(data, true);
}

// -----------------------------------------------------------------------------
void ViSimulator::setVisualizer(const std::shared_ptr<Visualizer>& visualizer)
{
  viz_ = visualizer;
  camera_->setVisualizer(viz_);
}

// -----------------------------------------------------------------------------
void ViSimulator::visualize(
    real_t dt,
    real_t marker_size_trajectory,
    real_t marker_size_landmarks)
{
  camera_->visualize(dt, marker_size_trajectory, marker_size_landmarks);
  viz_->drawCoordinateFrame("simulated_camera", 0u, T_W_Bk_, marker_size_trajectory * 5);
}

// -----------------------------------------------------------------------------
ViSimulator::Ptr createViSimulationScenario1()
{
  // Create trajectory:
  PoseSeries pose_series;
  pose_series.load(joinPath(getTestDataDir("ze_applanix_gt_data"), "traj_es.csv"));
  StampedTransformationVector poses = pose_series.getStampedTransformationVector();

  // Set start time of trajectory to zero.
  const int64_t offset = poses.at(0).first;
  for (StampedTransformation& it : poses)
  {
    it.first -= offset;
  }

  std::shared_ptr<BSplinePoseMinimalRotationVector> bs =
      std::make_shared<BSplinePoseMinimalRotationVector>(3);
  bs->initPoseSplinePoses(poses, 100, 0.5);
  TrajectorySimulator::Ptr trajectory = std::make_shared<SplineTrajectorySimulator>(bs);

  // Create camera simulation:
  CameraRig::Ptr rig = cameraRigFromYaml(joinPath(getTestDataDir("camera_models"),
                                                  "camera_rig_3.yaml"));
  // Create Vi-Simulator:
  CameraSimulatorOptions cam_sim_options;
  cam_sim_options.min_depth_m = 4.0;
  cam_sim_options.max_depth_m = 10.0;
  cam_sim_options.max_num_landmarks_ = 20000;
  ViSimulator::Ptr vi_sim =
      std::make_shared<ViSimulator>(trajectory, rig, cam_sim_options);
  vi_sim->initialize();
  return vi_sim;
}

} // namespace ze
