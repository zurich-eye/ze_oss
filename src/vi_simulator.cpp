#include <ze/vi_simulation/vi_simulator.hpp>

namespace ze {

// -----------------------------------------------------------------------------
ViSimulator::ViSimulator(
    const TrajectorySimulator::Ptr& trajectory,
    const CameraRig::Ptr& camera_rig,
    const CameraSimulatorOptions& camera_sim_options,
    const FloatType gyr_bias_noise_sigma,
    const FloatType acc_bias_noise_sigma,
    const FloatType gyr_noise_sigma,
    const FloatType acc_noise_sigma,
    const uint32_t cam_bandwidth_hz,
    const uint32_t imu_bandwidth_hz,
    const FloatType gravity_magnitude)
  : trajectory_(trajectory)
  , cam_dt_ns_(secToNanosec(1.0 / cam_bandwidth_hz))
  , imu_dt_ns_(secToNanosec(1.0 / imu_bandwidth_hz))
  , last_sample_stamp_(secToNanosec(trajectory->start()))
{
  CHECK(imu_bandwidth_hz % cam_bandwidth_hz == 0);

  ImuBiasSimulator::Ptr bias =
      std::make_shared<ContinuousBiasSimulator>(
        Vector3::Constant(gyr_bias_noise_sigma),
        Vector3::Constant(acc_bias_noise_sigma),
        trajectory->start(),
        trajectory->end(),
        (trajectory->end() - trajectory->start()) * imu_bandwidth_hz);

  imu_ = std::make_shared<ImuSimulator>(
           trajectory,
           bias,
           RandomVectorSampler<3>::sigmas(Vector3::Constant(acc_noise_sigma)),
           RandomVectorSampler<3>::sigmas(Vector3::Constant(gyr_noise_sigma)),
           imu_bandwidth_hz,
           imu_bandwidth_hz,
           gravity_magnitude);

  camera_ = std::make_shared<CameraSimulator>(
              trajectory,
              camera_rig,
              camera_sim_options);
}

// -----------------------------------------------------------------------------
std::pair<ViSensorData, bool> ViSimulator::getMeasurement()
{
  int64_t new_img_stamp = last_sample_stamp_ + cam_dt_ns_;

  ViSensorData data;
  if (nanosecToSecTrunc(new_img_stamp) > trajectory_->end())
  {
    LOG(WARNING) << "Reached end of trajectory!";
    return std::make_pair(data, false);
  }

  FloatType time_s = nanosecToSecTrunc(new_img_stamp);
  data.cam_timestamp_ = new_img_stamp;
  data.cam_measurements_ = camera_->getMeasurementsCorrupted(time_s);

  uint64_t imu_stamp = last_sample_stamp_;
  uint32_t num_imu_measurements = cam_dt_ns_ / imu_dt_ns_;
  data.imu_measurements_.resize(Eigen::NoChange, num_imu_measurements);
  data.imu_stamps_.resize(num_imu_measurements);
  for (uint32_t i = 0; i < num_imu_measurements; ++num_imu_measurements)
  {
    data.imu_stamps_(i) = imu_stamp;
    data.imu_measurements_.block<3,1>()

    //
    // TODO
    //

    imu_stamp += imu_dt_ns_;
  }
  CHECK_EQ(imu_stamp, new_img_stamp + imu_dt_ns_);




}

} // namespace ze
