#pragma once

#include <functional>
#include <ze/common/types.h>

// fwd
namespace cv {
class Mat;
}

namespace ze {
namespace data_provider {

using IMUCallback =
  std::function<void (int64_t& /*stamp*/, const Eigen::Vector3d& /*acc*/, const Eigen::Vector3d& /*gyr*/)>;

using CameraCallback =
  std::function<void (int64_t& /*stamp*/, const cv::Mat& /*img*/, size_t& /*camera-idx*/)>;

} // namespace data_provider
} // namespace ze
