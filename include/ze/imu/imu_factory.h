#pragma once

#include <memory>
#include <type_traits>

#include <Eigen/Dense>

#include <ze/imu/imu.h>
#include <ze/imu/imu_distortion.h>

namespace ze {

Imu::Ptr createImu(ImuDistortion::Type distortion_type);

}  // namespace ze
