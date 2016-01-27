#include <ze/imu/imu_calibration.h>

namespace ze {

void ImuCalibration::print(std::ostream& out, const std::string& s) const
{
  out << s << "\n"
      << "  delay_imu_cam = " << delay_imu_cam << "\n"
      << "  sigma_omega_c = " << gyro_noise_density << "\n"
      << "  sigma_acc_c = " << acc_noise_density << "\n"
      << "  sigma_omega_bias_c = " << gyro_bias_random_walk_sigma << "\n"
      << "  sigma_acc_bias_c = " << acc_bias_random_walk_sigma << "\n"
      << "  g = " << gravity_magnitude << "\n"
      << "  coriolis = " << omega_coriolis.transpose() << std::endl;
}

} // namespace ze
