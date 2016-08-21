#include <ze/imu_evaluation/pre_integration_runner.hpp>

namespace ze {

//------------------------------------------------------------------------------
PreIntegrationRunner::PreIntegrationRunner(ImuSimulator::Ptr scenario_runner,
                                           real_t imu_sampling_time,
                                           real_t camera_sampling_time)
  : scenario_runner_(scenario_runner)
  , imu_sampling_time_(imu_sampling_time)
  , camera_sampling_time_(camera_sampling_time)
  , initial_orientation_(Matrix3::Identity())
{
  if (camera_sampling_time_ >= 0)
  {
    CHECK_LE(imu_sampling_time_, camera_sampling_time_)
        << "IMU Sampling time must be smaller than camera's";
  }
}

//------------------------------------------------------------------------------
void PreIntegrationRunner::setInitialOrientation(Matrix3 initial_orientation)
{
  initial_orientation_ = initial_orientation;
}

//------------------------------------------------------------------------------
void PreIntegrationRunner::process(PreIntegrator::Ptr pre_integrator,
                                   bool corrupted,
                                   real_t start,
                                   real_t end)
{
  real_t t = start;
  pre_integrator->setInitialOrientation(initial_orientation_);

  // For negative camera sampling rates, a single batch of imu samples between
  // start and end is taken.
  real_t next_camera_sample;
  int samples_per_batch;

  std::vector<real_t> times;
  // Worst case container size allocation.
  if (camera_sampling_time_ < 0)
  {
    next_camera_sample = end;
    samples_per_batch =static_cast<int>(
                         std::ceil(end - start) / imu_sampling_time_) + 1;
  }
  else
  {
    next_camera_sample = start + camera_sampling_time_;
    samples_per_batch = static_cast<int>(
                          std::ceil(camera_sampling_time_ / imu_sampling_time_)) + 1;
  }

  ImuAccGyrContainer imu_measurements(6, samples_per_batch);

  int i = 0;
  for (real_t t = start; t <= end; t += imu_sampling_time_)
  {
    times.push_back(t);
    Vector6 measurement;
    if (corrupted)
    {
      measurement.head<3>() = scenario_runner_->specificForceCorrupted(t);
      measurement.tail<3>() = scenario_runner_->angularVelocityCorrupted(t);
    }
    else
    {
      measurement.head<3>() = scenario_runner_->specificForceActual(t);
      measurement.tail<3>() = scenario_runner_->angularVelocityActual(t);
    }
    imu_measurements.col(i) = measurement;

    // Wait for the next camera sample and push the collected data to the
    // integrator.
    if (t > next_camera_sample)
    {
      // Show the mean of the rotational velocities.
      Vector6 mean = imu_measurements.rowwise().mean();
      Vector6 max = imu_measurements.rowwise().maxCoeff();
      VLOG(1) << "Average Rotational Velocity: " << mean[3] << ", "
              << mean[4] << ", " << mean[5];
      VLOG(1) << "Rotational Velocity Max: " << max[3] << ", "
              << max[4] << ", " << max[4];

      next_camera_sample = t + camera_sampling_time_;
      pre_integrator->pushD_R_i_j(times, imu_measurements.leftCols(times.size()));
      times.clear();
      i = 0;
    }

    ++i;
  }

  // Ensure that all results are pushed to the container.
  if (imu_measurements.size() != 0)
  {
    // Show the mean of the rotational velocities.
    Vector6 mean = imu_measurements.rowwise().mean();
    Vector6 max = imu_measurements.rowwise().maxCoeff();
    VLOG(1) << "Average Rotational Velocity: " << mean[3] << ", "
            << mean[4] << ", " << mean[5];
    VLOG(1) << "Rotational Velocity Max: " << max[3] << ", "
            << max[4] << ", " << max[4];

    pre_integrator->pushD_R_i_j(times, imu_measurements.leftCols(times.size()));
  }
}

//------------------------------------------------------------------------------
PreIntegrationRunnerDataProvider::PreIntegrationRunnerDataProvider(
    DataProviderBase::Ptr data_provider)
  : data_provider_(data_provider)
  , initial_orientation_(Matrix3::Identity())
{}

//------------------------------------------------------------------------------
void PreIntegrationRunnerDataProvider::setInitialOrientation(
    Matrix3 initial_orientation)
{
  initial_orientation_ = initial_orientation;
}

//------------------------------------------------------------------------------
void PreIntegrationRunnerDataProvider::loadData()
{
  if (times_.size() == 0)
  {
    std::vector<real_t> times;
    std::vector<Vector6> measurement_vector;

    // subscribe to the dataprovider callback to get all measurements between
    // start and end.
    ImuCallback integrate = [&times, &measurement_vector](
                            int64_t stamp, const Vector3& acc,
                            const Vector3 gyr, uint32_t imu_idx)
    {
      real_t stamp_float = static_cast<FloatType>(stamp) * 1e-9;

      Vector6 measurement;
      // @todo: actually inject into integrators
      // The below values are the actual values available in the EUROC MH01
      // Dataset.
      Vector3 gyr_bias;
      gyr_bias << -0.002133, 0.021059, 0.076659;

      measurement.head<3>() = acc; // @todo: remove bias
      measurement.tail<3>() = gyr - gyr_bias;

      measurement_vector.push_back(measurement);
      times.push_back(stamp_float);
    };

    // Subscribe dataprovider and collect imu measurements.
    data_provider_->registerImuCallback(integrate);
    data_provider_->spin();

    // Map onto an acc/gyr container.
    times_ = times;
    imu_measurements_.resize(6, measurement_vector.size());
    for (size_t i = 0; i < measurement_vector.size(); ++i)
    {
      imu_measurements_.col(i) = measurement_vector[i];
    }
  }
}

//------------------------------------------------------------------------------
void PreIntegrationRunnerDataProvider::process(
    PreIntegrator::Ptr pre_integrator)
{
  loadData();

  // The actual preintegration.
  pre_integrator->setInitialOrientation(initial_orientation_);
  pre_integrator->pushD_R_i_j(times_, imu_measurements_.leftCols(times_.size()));
}

} // namespace
