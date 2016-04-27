#pragma once

namespace ze {
namespace cu {

enum class StereoPDSolver
{
  HuberL1, //!< Huber regularization + pointwise L1 intensity matching costs
  PrecondHuberL1, //!< Huber regularization + pointwise L1 intensity matching costs
  PrecondHuberL1Weighted, //!< weighted Huber regularization + pointwise L1 intensity matching costs
  EpipolarPrecondHuberL1 //!< Huber regularization + pointwise L1 intensity matching costs applied on generic images with known epipolar geometry (not rectified)
};

} // namespace cu
} // namespace ze

