#pragma once

namespace ze {

// -----------------------------------------------------------------------------
class PreIntegrationEvaluationNode
{
public:
  PreIntegrationEvaluationNode();
  ~PreIntegrationEvaluationNode() = default;

  //! Return the ImuBias for model and parameters specified in the GFlags.
  ImuBias::Ptr imuBias(FloatType start, FloatType end);

  //! Monte-Carlo Simulation Runners:
  //! Pass an optional MonteCarlo runner to re-use these results
  //! instead of re-running the simulation.
//  PreIntegratorMonteCarlo::Ptr runManifoldClean(
//      PreIntegrationRunner::Ptr preintegration_runner,
//      PreIntegrator::IntegratorType integrator_type,
//      const std::string& name,
//      const Matrix3& gyroscope_noise_covariance,
//      const std::vector<Matrix3>* D_R_i_k_reference,
//      FloatType start,
//      FloatType end);

  PreIntegratorMonteCarlo::Ptr runManifoldCorruptedMc(
      PreIntegrationRunner::Ptr preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      FloatType start,
      FloatType end);

  PreIntegratorMonteCarlo::Ptr runQuaternionMc(
      PreIntegrationRunner::Ptr preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      FloatType start,
      FloatType end);

  //! Pre-Integration Simulation Runners:
  ManifoldPreIntegrationState::Ptr runManifoldCorrupted(
      PreIntegrationRunner::Ptr preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      FloatType start,
      FloatType end,
      PreIntegratorMonteCarlo::Ptr mc_ref);

  QuaternionPreIntegrationState::Ptr runQuaternion(
      PreIntegrationRunner::Ptr preintegration_runner,
      PreIntegrator::IntegratorType integrator_type,
      const std::string& name,
      const Matrix3& gyroscope_noise_covariance,
      FloatType start,
      FloatType end,
      PreIntegratorMonteCarlo::Ptr mc_ref);


  //! Runs a single noisy integration on all channels and shows plots of the
  //! estimates.
  //! Returns the error offset (rotation only) for the different methods
  std::vector<FloatType> runDriftEvaluation(
      PreIntegrationRunner::Ptr preintegration_runner,
      const Matrix3& gyroscope_noise_covariance,
      FloatType start,
      FloatType end,
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

  //! Plot the estimated / integrated vector of rotation matrices.
  void plotOrientation(const std::vector<FloatType>& times,
                       const std::vector<Matrix3>& R_i,
                       const std::string& label,
                       const bool plot_reference = false);

  void plotOrientationError(const std::vector<FloatType>& times,
                              const std::vector<Matrix3>& est,
                              const std::string& label);

  //! Plot the measurements used to obtain a preintegration state.
  void plotImuMeasurements(const std::vector<FloatType>& times,
                           const ImuAccGyrContainer& measurements,
                           const ImuAccGyrContainer& measurements2);

  void shutdown();

private:
  ImuPreIntegrationParameters parameters_;

  std::shared_ptr<BSplinePoseMinimalRotationVector> trajectory_;

  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<SplinesVisualizer> splines_visualizer_;
};

} // namespace ze
