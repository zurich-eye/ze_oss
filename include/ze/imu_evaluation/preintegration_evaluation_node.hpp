#pragma once

namespace ze {

// -----------------------------------------------------------------------------
class PreIntegrationEvaluationNode
{
public:
  PreIntegrationEvaluationNode();
  ~PreIntegrationEvaluationNode() = default;

  //! Run monte carlo simulations to evaluate the covariance propagation.
  void runCovarianceMonteCarloMain();

  //! Run various integration methods to evaluate drift
  void runDriftEvaluationMain();

  //! Run pre-integration on a given dataset.
  void runRealDatasetMain();

  //! Monte-Carlo Simulation Runners:
  //! Pass an optional MonteCarlo runner to re-use these results
  //! instead of re-running the simulation.
//  PreIntegratorMonteCarlo::Ptr runManifoldClean(
//      PreIntegrationRunner::Ptr preintegration_runner,
//      PreIntegrator::IntegratorType integrator_type,
//      const std::string& name,
//      const Matrix3& gyroscope_noise_covariance,
//      const std::vector<Matrix3>* D_R_i_k_reference,
//      real_t start,
//      real_t end);

  PreIntegratorMonteCarlo::Ptr runManifoldCorruptedMc(
      const PreIntegrationRunner::Ptr& preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      real_t start,
      real_t end);

  PreIntegratorMonteCarlo::Ptr runQuaternionMc(
      const PreIntegrationRunner::Ptr& preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      real_t start,
      real_t end);

  //! Pre-Integration Simulation Runners:
  ManifoldPreIntegrationState::Ptr runManifoldCorrupted(
      const PreIntegrationRunner::Ptr& preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      real_t start,
      real_t end,
      const PreIntegratorMonteCarlo::Ptr& mc_ref,
      bool simplified_covariance = false);

  QuaternionPreIntegrationState::Ptr runQuaternion(
      const PreIntegrationRunner::Ptr& preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      real_t start,
      real_t end,
      const PreIntegratorMonteCarlo::Ptr& mc_ref);

  //! Runs a single noisy integration on all channels and shows plots of the
  //! estimates.
  //! Returns the error offset (rotation only) for the different methods
  std::vector<real_t> runDriftEvaluation(
      const PreIntegrationRunner::Ptr& preintegration_runner,
      const Matrix3& gyroscope_noise_covariance,
      real_t start,
      real_t end,
      bool plot = false);

  void runDriftEvaluationRuns(size_t runs,
                              const Matrix3& gyroscope_noise_covariance,
                              RandomVectorSampler<3>::Ptr accel_noise,
                              RandomVectorSampler<3>::Ptr gyro_noise);

  //! Get a preintegration runner and generate a trajectory.
  PreIntegrationRunner::Ptr getPreIntegrationRunner(
      RandomVectorSampler<3>::Ptr accel_noise,
      RandomVectorSampler<3>::Ptr gyro_noise);

  //! Given the GFLags configuration loads and returns a bspline representing
  //! a trajectory.
  void loadTrajectory();

  //! Show the trajectory that was loaded or generated.
  void showTrajectory();

  //! Generate a series of plots that show the results
  void plotCovarianceResults(
      std::initializer_list<const std::vector<Matrix3>> covariances_vectors,
      std::initializer_list<const std::string> labels);

  //! Generate a series of plots that show the results
  void plotCovarianceError(const std::vector<Matrix3> ref,
                             const std::vector<Matrix3> est,
                             const std::string& label);

  void shutdown();

private:
  ImuPreIntegrationParameters parameters_;

  std::shared_ptr<BSplinePoseMinimalRotationVector> trajectory_;

  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<SplinesVisualizer> splines_visualizer_;
};

} // namespace ze
