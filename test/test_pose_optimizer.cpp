#include <random>
#include <ze/common/test/entrypoint.h>
#include <ze/common/timer.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/nlls_solver/least_squares_solver.h>
#include <ze/nlls_solver/robust_cost.h>

namespace ze {

class PoseOptimizer : public LeastSquaresSolver<6, Transformation, PoseOptimizer>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;

  PoseOptimizer(const Keypoints& measurements,
                const Positions& landmarks,
                const Transformation& T_C_B,
                const PinholeCamera& cam,
                Transformation& T_B_W)
    : u_(measurements)
    , p_W_(landmarks)
    , T_C_B_(T_C_B)
    , cam_(cam)
  {}

  double evaluateError(const Transformation& T_B_W, HessianMatrix* H,
                       GradientVector *g)
  {
    // Transform points from world coordinates to camera coordinates.
    Positions p_C = (T_C_B_ * T_B_W).transformVectorized(p_W_);

    // Normalize points to obtain estimated bearing vectors.
    Keypoints u_est = cam_.projectVectorized(p_C);

    // Compute difference between bearing vectors.
    Keypoints u_err = u_ - u_est;

    Eigen::VectorXd u_err_norm = u_err.colwise().norm();

    //
    // TODO: Implement robust cost function that takes vector of errors.
    //

    // Projection jacobian tranposed:
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_proj_tr =
        cam_.dProject_dBearingVectorized(p_C);

    // Compute Jacobian w.r.t. body
    //
    // TODO
    //

    return 0.0;
  }

  void update(const Transformation& T_Bold_W, const UpdateVector& dx,
              Transformation& T_Bnew_W)
  {
    T_Bnew_W = Transformation::exp(dx)*T_Bold_W;
  }

private:
  const Keypoints& u_;  ///< Bearing vectors corresponding to feature measurements.
  const Positions& p_W_;    ///< 3D points corresponding to features.
  const Transformation& T_C_B_;   ///< Camera-IMU extrinsic calibration.
  const PinholeCamera& cam_;
};

} // namespace ze


TEST(NllsPoseOptimizerTests, testSolver)
{
  using namespace ze;

  Transformation T_C_B, T_B_W;
  T_C_B.setRandom(); // Random camera to imu/body transformation.
  T_B_W.setRandom(); // Random body transformation.

  const size_t n = 200;
  PinholeCamera cam(640, 480, 329.11, 329.11, 320.0, 240.0);
  Keypoints pix_true = generateRandomKeypoints(640, 480, 10, n);

  Positions pos_C = cam.backProjectVectorized(pix_true);

  // Obtain the 3D points by applying a random scaling between 1 and 3 meters.
  std::ranlux24 gen;
  std::uniform_real_distribution<double> scale(1.0, 3.0);
  for(size_t i = 0; i < n; ++i) {
    pos_C.col(i) *= scale(gen);
  }

  // Transform points to world coordinates.
  Positions pos_W = (T_B_W.inverse() * T_C_B.inverse()).transformVectorized(pos_C);

  // Apply some noise to the keypoints to simulate measurements.
  Keypoints pix_noisy = pix_true;
  const double stddev = 1.0;
  std::normal_distribution<double> px_noise(0.0, stddev);
  for(size_t i = 0; i < n; ++i) {
    pix_noisy(0,i) += px_noise(gen);
    pix_noisy(1,i) += px_noise(gen);
  }

  ze::Timer t;
  PoseOptimizer optimizer(pix_noisy, pos_W, T_C_B, cam, T_B_W);
  for(size_t i = 0; i < 10; ++i)
    optimizer.evaluateError(T_B_W, nullptr, nullptr);
  std::cout << "evaluateError took " << t.stop() * 1000 << " ms\n";
}


ZE_UNITTEST_ENTRYPOINT
