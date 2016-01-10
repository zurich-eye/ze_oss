#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace ze {

enum class StrategyType {
  GaussNewton,
  LevenbergMarquardt
};

struct LeastSquaresSolverOptions
{
  /// Solver strategy.
  StrategyType strategy = StrategyType::GaussNewton;

  /// Damping parameter. If mu > 0, coefficient matrix is positive definite, this
  /// ensures that x is a descent direction. If mu is large, x is a short step in
  /// the steepest direction. This is good if the current iterate is far from the
  /// solution. If mu is small, LM approximates gauss newton iteration and we
  /// have (almost) quadratic convergence in the final stages.
  double mu_init = 0.01f;

  /// Increase factor of mu after fail
  double nu_init = 2.0;

  /// Max number of iterations
  size_t max_iter = 15;

  /// Max number of trials (used in LevenbergMarquardt)
  size_t max_trials = 5;

  /// Stop when error increases.
  bool stop_when_error_increases = false;

  /// Output Statistics
  bool verbose = false;

  /// Stop if update norm is smaller than eps
  double eps = 0.0000000001;
};

/// Abstract Class for solving nonlinear least-squares (NLLS) problems.
/// Template Parameters: D  : dimension of the state, T: type of the model
/// e.g. SE2, SE3
template <int D, typename T, typename Implementation>
class LeastSquaresSolver
{
public:
  typedef T State;
  typedef Eigen::Matrix<double, D, D> HessianMatrix;
  typedef Eigen::Matrix<double, D, 1> GradientVector;
  typedef Eigen::Matrix<double, D, 1> UpdateVector;

  LeastSquaresSolverOptions solver_options_;

protected:
  LeastSquaresSolver() = default;

  LeastSquaresSolver(const LeastSquaresSolverOptions& options);

  virtual ~LeastSquaresSolver() = default;

public:
  /// Calls the GaussNewton or LevenbergMarquardt optimization strategy
  void optimize(State& state);

  /// Gauss Newton optimization strategy
  void optimizeGaussNewton(State& state);

  /// Levenberg Marquardt optimization strategy
  void optimizeLevenbergMarquardt(State& state);

  /// Reset all parameters to restart the optimization
  void reset();

  /// Get the squared error
  inline double getError() const
  {
    return chi2_;
  }

  /// The the Hessian matrix (Information Matrix).
  inline const Eigen::Matrix<double, D, D>& getHessian() const
  {
    return H_;
  }

protected:

  Implementation& impl()
  {
    return *static_cast<Implementation*>(this);
  }

  /// Evaluates the error at provided state. Optional return variables are
  /// the Hessian matrix and the gradient vector (Jacobian * residual).
  /// If these parameters are requested, the system is linearized at the current
  /// state.
  double evaluateError(
      const State& state,
      HessianMatrix* H,
      GradientVector* g)
  {
    return impl().evaluateError(state, H, g);
  }

  /// Solve the linear system H*dx = g to obtain optimal perturbation dx.
  bool solve(
      const State& state,
      const HessianMatrix& H,
      const GradientVector& g,
      UpdateVector& dx)
  {
    if(&LeastSquaresSolver::solve != &Implementation::solve)
      return impl().solve(state, H, g, dx);
    else
      return solveDefaultImpl(H, g, dx);
  }

  /// Apply the perturbation dx to the state.
  void update(
      const State& state,
      const UpdateVector& dx,
      State& new_state)
  {
    impl().update(state, dx, new_state);
  }

  void startIteration()
  {
    if(&LeastSquaresSolver::startIteration != &Implementation::startIteration)
      impl().startIteration();
  }

  void finishIteration()
  {
    if(&LeastSquaresSolver::finishIteration != &Implementation::finishIteration)
      impl().finishIteration();
  }

  void finishTrial()
  {
    if(&LeastSquaresSolver::finishTrial != &Implementation::finishTrial)
      impl().finishTrial();
  }

private:

  /// Default implementation to solve the linear system H*dx = g to obtain optimal perturbation dx.
  bool solveDefaultImpl(
      const HessianMatrix& H,
      const GradientVector& g,
      UpdateVector& dx);

protected:
  HessianMatrix  H_;        ///< Hessian or approximation Jacobian*Jacobian^T.
  GradientVector g_;        ///< Jacobian*residual.
  UpdateVector   dx_;       ///< Update step.
  double chi2_ = 0.0;       ///< Whitened error / log-likelihood: 1/(2*sigma^2)*(z-h(x))^2.
  double rho_ = 0.0;        ///< Error reduction: chi2-new_chi2.
  double mu_ = 0.01;        ///< Damping parameter.
  double nu_ = 2.0;         ///< Factor that specifies how much we increase mu at every trial.
  bool stop_ = false;       ///< Stop flag.
  size_t iter_ = 0;         ///< Current Iteration.
  size_t trials_ = 0;       ///< Current number of trials.
};

} // namespace ze

#include <ze/geometry/least_squares_solver-inl.h>
