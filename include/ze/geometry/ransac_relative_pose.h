// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/types.h>
#include <ze/common/transformation.h>

namespace ze {

// fwd
class Camera;

using BearingsVector = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

// utility
inline BearingsVector bearingsVectorFromBearings(const Bearings& f)
{
  BearingsVector v;
  v.reserve(f.cols());
  for (int i = 0; i < f.cols(); ++i)
  {
    v.push_back(f.col(i).cast<double>());
  }
  return v;
}

enum class RelativePoseAlgorithm {
  FivePoint,               //!< Nister 5-point relative pose.
  TwoPointTranslationOnly, //!< 2-point relative pose, assumes known rotation between frames.
  TwoPointRotationOnly     //!< 2-point relative pose, assumes no translation between frames.
};

class RansacRelativePose
{
public:
  RansacRelativePose() = delete;

  RansacRelativePose(
      const Camera& cam,
      const real_t& reprojection_threshold_px);

  bool solve(
      const Bearings& f_ref,
      const Bearings& f_cur,
      const RelativePoseAlgorithm method,
      Transformation& T_cur_ref);

  bool solve(
      const BearingsVector& f_ref,
      const BearingsVector& f_cur,
      const RelativePoseAlgorithm method,
      Transformation& T_cur_ref);

  bool solveRelativePose(
      const BearingsVector& f_ref,
      const BearingsVector& f_cur,
      Transformation& T_cur_ref);

  bool solveTranslationOnly(
      const BearingsVector& f_ref,
      const BearingsVector& f_cur,
      Transformation& T_cur_ref);

  bool solveRotationOnly(
      const BearingsVector& f_ref,
      const BearingsVector& f_cur,
      Transformation& T_cur_ref);

  inline uint32_t numIterations() const { return num_iterations_; }

  inline const std::vector<int>& inliers() const { return inliers_; }

  std::vector<int> outliers();

  //! @name: OpenGV settings.
  //! @{
  real_t ogv_threshold_;
  uint32_t ogv_max_iterations_ = 100;
  real_t ogv_init_probability_ = 0.999;
  uint32_t ogv_verbosity_level_ = 0u;
  //! @}

private:
  uint32_t num_measurements_ = 0u;
  uint32_t num_iterations_ = 0u;
  real_t result_probability_;
  std::vector<int> inliers_;
};

} // namespace ze
