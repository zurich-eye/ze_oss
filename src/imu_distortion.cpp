#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ze/imu/imu_distortion.h>

namespace ze {

ImuDistortion::ImuDistortion(ImuDistortion::Type distortion_type)
  : distortion_type_(distortion_type)
{}

}  // namespace ze
