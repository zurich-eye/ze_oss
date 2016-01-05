#ifndef STEREO_SOLVER_ENUM_HPP
#define STEREO_SOLVER_ENUM_HPP

namespace imp {
namespace cu {

enum class StereoPDSolver
{
  HuberL1, //!< Huber regularization + pointwise L1 intensity matching costs
  PrecondHuberL1, //!< Huber regularization + pointwise L1 intensity matching costs
  PrecondHuberL1Weighted, //!< weighted Huber regularization + pointwise L1 intensity matching costs
  EpipolarPrecondHuberL1 //!< Huber regularization + pointwise L1 intensity matching costs applied on generic images with known epipolar geometry (not rectified)
};

} // namespace cu
} // namespace imp

#endif // STEREO_SOLVER_ENUM_HPP

