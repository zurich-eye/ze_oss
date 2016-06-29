#pragma once

namespace ze {

DEFINE_string(trajectory_source, "", "Path to file to load curve from, default: generate random curve");

DEFINE_int32(monte_carlo_runs, 10, "Number of monte-carlo simulations.");

// For an empty curve_source parameters to generate a random trajectory:
DEFINE_double(trajectory_length, 30.0, "The length of the curve in seconds");
DEFINE_int32(trajectory_interpolation_points, 100, "Number of interpolation to randomize");
DEFINE_int32(trajectory_num_segments, 100, "Number of spline segments to fit the interpolation points.");
DEFINE_double(trajectory_lambda, 1e-5, "The regularizing smoothing factor used to fit the trajectory to the interpolation curve. Smaller values give more aggressive curves.");

// Imu Simulation Parameters:
DEFINE_double(imu_sampling_time, 0.005, "Sampling time of the IMU.");
DEFINE_double(camera_sampling_time, 0.1, "Sampling time of the Camera, only used to split the pre-integration steps.");
DEFINE_double(accelerometer_noise_density, 0, "Noise density of accelerometer.");
DEFINE_double(gyroscope_noise_density, 0, "Noise density of gyroscope");

// Im u Bias Simulation
DEFINE_string(imu_bias_type, "constant", "Bias model of the imu");
DEFINE_double(imu_acc_bias_noise_density, 0, "Noise density of continuous-time accelerometer bias.");
DEFINE_double(imu_gyr_bias_noise_density, 0, "Noise density of continuous-time gyroscope bias.");

DEFINE_double(imu_acc_bias_const, 0, "Value of constant accelerometer bias.");
DEFINE_double(imu_gyr_bias_const, 0, "Value of constant gyroscope bias.");

// A series of visualization controls:
DEFINE_bool(show_trajectory, true, "Show the trajectory that was loaded / generated.");
DEFINE_int32(num_threads, 16, "Number of threads to take for the monte-carlo simulations");

//! A parameter structure to regroup all simulation parameters.
struct ImuPreIntegrationParameters
{
  // The Imu Model and Noise Parameters:
  FloatType imu_sampling_time;
  FloatType camera_sampling_time;
  FloatType accel_noise_bandwidth_hz;
  FloatType gyro_noise_bandwidth_hz;
  FloatType accel_noise_density;
  FloatType gyro_noise_density;
  Vector3 gravity;

  // Imu Bias Model
  std::string imu_bias_type;

  // Parameters of continous-time bias model:
  FloatType imu_acc_bias_noise_density;
  FloatType imu_gyr_bias_noise_density;

  // Parameters of constant bias model:
  FloatType imu_acc_bias_const;
  FloatType imu_gyr_bias_const;

  // Trajectory Generation Parameters:
  std::string trajectory_source;
  FloatType trajectory_start_time;
  FloatType trajectory_end_time;
  int trajectory_num_interpolation_points;
  int trajectory_num_segments;
  FloatType trajectory_lambda;
  int trajectory_spline_order = 10;

  //! Initialize a parameter structure from gflags
  static ImuPreIntegrationParameters fromGFlags()
  {
    ImuPreIntegrationParameters p;

    p.imu_sampling_time = FLAGS_imu_sampling_time;
    p.camera_sampling_time = FLAGS_camera_sampling_time;
    p.accel_noise_bandwidth_hz = FLAGS_accelerometer_noise_bandwidth_hz;
    p.gyro_noise_bandwidth_hz = FLAGS_gyroscope_noise_bandwidth_hz;
    p.gravity = Vector3(0, 0, -9.81);

    p.accel_noise_density = FLAGS_accelerometer_noise_density;
    p.gyro_noise_density = FLAGS_gyroscope_noise_density;

    p.trajectory_source = FLAGS_trajectory_source;
    p.trajectory_start_time = 0.0;
    p.trajectory_end_time = FLAGS_trajectory_length;
    p.trajectory_num_interpolation_points = FLAGS_trajectory_interpolation_points;
    p.trajectory_num_segments = FLAGS_trajectory_num_segments;
    p.trajectory_lambda = FLAGS_trajectory_lambda;

    p.imu_bias_type = FLAGS_imu_bias_type;

    p.imu_acc_bias_noise_density = FLAGS_imu_acc_bias_noise_density;
    p.imu_gyr_bias_noise_density = FLAGS_imu_gyr_bias_noise_density;

    p.imu_acc_bias_const = FLAGS_imu_acc_bias_const;
    p.imu_gyr_bias_const = FLAGS_imu_gyr_bias_const;

    return p;
  }

  //! Initialize a parameter structure given an imu yaml file.
  static /*ImuPreIntegrationParameters*/ void fromImuModel()
  {
    //! @todo
  }
};

} // namespace ze
