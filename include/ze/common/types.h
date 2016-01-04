#pragma once

#include <cstdint>
#include <Eigen/Core>

namespace ze {

//------------------------------------------------------------------------------
// Scalars and fp precision.
using size_t    = std::size_t;
using int64_t   = std::int64_t;   
using uint8_t   = std::uint8_t;
using uint64_t  = std::uint64_t;
using FloatType = double;

//------------------------------------------------------------------------------
// Feature containers.
using Keypoint = Eigen::Matrix<FloatType, 2, 1>;
using Bearing = Eigen::Matrix<FloatType, 3, 1>;
using Position = Eigen::Matrix<FloatType, 3, 1>;
using Gradient = Eigen::Matrix<FloatType, 2, 1>;
using Keypoints = Eigen::Matrix<FloatType, 2, Eigen::Dynamic, Eigen::ColMajor>;
using Bearings = Eigen::Matrix<FloatType, 3, Eigen::Dynamic, Eigen::ColMajor>;
using Positions = Eigen::Matrix<FloatType, 3, Eigen::Dynamic, Eigen::ColMajor>;
using Gradients = Eigen::Matrix<FloatType, 2, Eigen::Dynamic, Eigen::ColMajor>;

} // namespace ze
