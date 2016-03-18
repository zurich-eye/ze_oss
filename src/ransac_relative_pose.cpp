#include <ze/geometry/ransac_relative_pose.h>

#include <glog/logging.h>

#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/sac_problems/relative_pose/RotationOnlySacProblem.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/triangulation/methods.hpp>

#include <ze/cameras/camera.h>

namespace ze {

RansacRelativePose::RansacRelativePose(
    const Camera& cam,
    const FloatType& reprojection_threshold_px)
  : opengv_threshold_(
      1.0 - std::cos(cam.getApproxAnglePerPixel() * reprojection_threshold_px))
{}

bool RansacRelativePose::solve(
      const Bearings& f_ref,
      const Bearings& f_cur,
      const RelativePoseAlgorithm method,
      Transformation& T_cur_ref)
{
  BearingsVector f_ref_v = bearingsVectorFromBearings(f_ref);
  BearingsVector f_cur_v = bearingsVectorFromBearings(f_cur);
  return solve(f_ref_v, f_cur_v, method, T_cur_ref);
}

bool RansacRelativePose::solve(
    const BearingsVector& f_ref,
    const BearingsVector& f_cur,
    const RelativePoseAlgorithm method,
    Transformation& T_cur_ref)
{
  CHECK_EQ(f_ref.size(), f_cur.size());
  switch(method)
  {
    case RelativePoseAlgorithm::FivePoint:
      return solveRelativePose(f_ref, f_cur, T_cur_ref);
      break;
    case RelativePoseAlgorithm::TwoPointTranslationOnly:
      return solveTranslationOnly(f_ref, f_cur, T_cur_ref);
      break;
    case RelativePoseAlgorithm::TwoPointRotationOnly:
      return solveRotationOnly(f_ref, f_cur, T_cur_ref);
      break;
    default:
      LOG(FATAL) << "Algorithm not implemented";
  }
  return false;
}

// -----------------------------------------------------------------------------
bool RansacRelativePose::solveRelativePose(
    const BearingsVector& f_ref,
    const BearingsVector& f_cur,
    Transformation& T_cur_ref)
{
  // Setup problem.
  using Problem = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  using Adapter = opengv::relative_pose::CentralRelativeAdapter;
  Adapter adapter(f_cur, f_ref);
  boost::shared_ptr<Problem> problem(new Problem(adapter, Problem::NISTER));
  opengv::sac::Ransac<Problem> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = opengv_threshold_;
  ransac.max_iterations_ = max_iterations_;
  ransac.probability_ = init_probability_;

  // Solve.
  if(!ransac.computeModel(verbosity_level_))
  {
    LOG(WARNING) << "5Pt RANSAC could not find a solution";
    return false;
  }
  VLOG(3) << "5Pt RANSAC:"
          << ", #iter = " << ransac.iterations_
          << ", #inliers = " << ransac.inliers_.size();

  // Process results.
  Matrix3 R = ransac.model_coefficients_.leftCols<3>();
  Vector3 t = ransac.model_coefficients_.rightCols<1>();
  T_cur_ref = Transformation(Quaternion(R), t);
  result_probability_ = ransac.probability_;
  num_iterations_ = ransac.iterations_;
  inliers_ = ransac.inliers_;
  return true;
}

// -----------------------------------------------------------------------------
bool RansacRelativePose::solveTranslationOnly(
    const BearingsVector& f_ref,
    const BearingsVector& f_cur,
    Transformation& T_cur_ref)
{
  // Setup Problem.
  using Problem = opengv::sac_problems::relative_pose::TranslationOnlySacProblem;
  using Adapter = opengv::relative_pose::CentralRelativeAdapter;
  Adapter adapter(f_cur, f_ref, T_cur_ref.getRotationMatrix());
  boost::shared_ptr<Problem> problem(new Problem(adapter));
  opengv::sac::Ransac<Problem> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = opengv_threshold_;
  ransac.max_iterations_ = max_iterations_;
  ransac.probability_ = init_probability_;

  // Solve.
  if(!ransac.computeModel(verbosity_level_))
  {
    LOG(WARNING) << "2Pt RANSAC could not find a solution";
    return false;
  }
  VLOG(3) << "2pt RANSAC:"
          << ", #iter = " << ransac.iterations_
          << ", #inliers = " << ransac.inliers_.size();

  // Process results.
  Matrix3 R = ransac.model_coefficients_.leftCols<3>();
  Vector3 t = ransac.model_coefficients_.rightCols<1>();
  T_cur_ref = Transformation(Quaternion(R), t);
  result_probability_ = ransac.probability_;
  num_iterations_ = ransac.iterations_;
  inliers_ = ransac.inliers_;
  return true;
}

// -----------------------------------------------------------------------------
bool RansacRelativePose::solveRotationOnly(
    const BearingsVector& f_ref,
    const BearingsVector& f_cur,
    Transformation& T_cur_ref)
{
  // Setup Problem.
  using Problem = opengv::sac_problems::relative_pose::RotationOnlySacProblem;
  opengv::relative_pose::CentralRelativeAdapter adapter(f_cur, f_ref);
  boost::shared_ptr<Problem> problem(new Problem(adapter));
  opengv::sac::Ransac<Problem> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = opengv_threshold_;
  ransac.max_iterations_ = max_iterations_;
  ransac.probability_ = init_probability_;

  // Solve.
  if(!ransac.computeModel(verbosity_level_))
  {
    LOG(WARNING) << "2Pt RANSAC could not find a solution";
    return false;
  }
  VLOG(3) << "2pt Rotation-Only RANSAC:"
          << ", #iter = " << ransac.iterations_
          << ", #inliers = " << ransac.inliers_.size();

  // Process results.
  Matrix3 R = ransac.model_coefficients_.leftCols<3>();
  Vector3 t = ransac.model_coefficients_.rightCols<1>();
  T_cur_ref = Transformation(Quaternion(R), t);

  result_probability_ = ransac.probability_;
  num_iterations_ = ransac.iterations_;
  inliers_ = ransac.inliers_;
  return true;
}

} // namespace ze
