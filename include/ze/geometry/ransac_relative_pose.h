#pragma once

#include <ze/common/types.h>
#include <ze/common/transformation.h>

namespace ze {

// fwd
class Camera;

using BearingsVector = std::vector<Vector3, Eigen::aligned_allocator<Vector3>>;

// utility
inline BearingsVector bearingsVectorFromBearings(const Bearings& f)
{
  const Vector3* data = reinterpret_cast<const Vector3*>(f.data());
  BearingsVector v(data, data + f.cols());
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
      const FloatType& reprojection_threshold_px);

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
  FloatType ogv_threshold_;
  uint32_t ogv_max_iterations_ = 100;
  FloatType ogv_init_probability_ = 0.999;
  uint32_t ogv_verbosity_level_ = 0u;
  //! @}

private:
  uint32_t num_measurements_ = 0u;
  uint32_t num_iterations_ = 0u;
  FloatType result_probability_;
  std::vector<int> inliers_;
};

} // namespace ze
