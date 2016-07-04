#include <ze/imu_evaluation/quaternion_pre_integrator.hpp>

namespace ze {

//------------------------------------------------------------------------------
QuaternionPreIntegrationState::QuaternionPreIntegrationState(
    Matrix3 gyro_noise_covariance,
    PreIntegrator::IntegratorType integrator_type)
  : PreIntegrator(gyro_noise_covariance, integrator_type)
{
  D_R_i_j_quat_.push_back(Quaternion());
  D_R_i_k_quat_.push_back(Quaternion());

  R_i_j_quat_.push_back(Quaternion());
  R_i_k_quat_.push_back(Quaternion());
}

//------------------------------------------------------------------------------
void QuaternionPreIntegrationState::setInitialOrientation(
                                      Matrix3 initial_orientation)
{
  PreIntegrator::setInitialOrientation(initial_orientation);
  R_i_j_quat_.clear();
  R_i_k_quat_.clear();
  R_i_j_quat_.push_back(Quaternion(initial_orientation));
  R_i_k_quat_.push_back(Quaternion(initial_orientation));
}

//------------------------------------------------------------------------------
void QuaternionPreIntegrationState::doPushD_R_i_j(
    times_container_t stamps,
    measurements_container_t measurements)
{
  size_t previous_items = D_R_i_k_quat_.size();

  switch (integrator_type_)
  {
    case PreIntegrator::FirstOrderForward:
      doPushFirstOrderFwd(stamps, measurements);
      break;
    case PreIntegrator::FirstOrderMidward:
      doPushFirstOrderMid(stamps, measurements);
      break;
    case PreIntegrator::RungeKutta3:
      doPushRK(stamps, measurements, 3);
      break;
    case PreIntegrator::RungeKutta4:
      doPushRK(stamps, measurements, 4);
      break;
    case PreIntegrator::CrouchGrossman3:
      doPushCrouchGrossman(stamps, measurements, 3);
      break;
    case PreIntegrator::CrouchGrossman4:
      doPushCrouchGrossman(stamps, measurements, 4);
      break;
    default:
      throw std::runtime_error("No valid integrator supplied");
  }

  // Push all quaternion values as rotation matrices (the base-reference type)
  for (size_t i = previous_items; i < D_R_i_k_quat_.size(); ++i)
  {
    if (compute_absolutes_)
    {
      R_i_k_.push_back(R_i_k_quat_[i].getRotationMatrix());
    }
    D_R_i_k_.push_back(D_R_i_k_quat_[i].getRotationMatrix());
  }

  times_raw_.push_back(stamps.back());

  //// Update the steps:
  // Explicitly push beginning if times_ is empty.
  if (times_.size() == 0)
  {
    times_.push_back(times_raw_.front());
  }
  times_.push_back(times_raw_.back());

  // Push the keyframe sampled pre-integration states:
  D_R_i_j_quat_.push_back(D_R_i_k_quat_.back());
  R_i_j_quat_.push_back(R_i_j_quat_.back() * D_R_i_j_quat_.back());
  D_R_i_j_.push_back(D_R_i_j_quat_.back().getRotationMatrix());
  R_i_j_.push_back(R_i_j_quat_.back().getRotationMatrix());

  // push covariance
  covariance_i_j_.push_back(covariance_i_k_.back());

  // Sanity Checks:
  CHECK_EQ(R_i_k_.size(), R_i_k_quat_.size());
  CHECK_EQ(D_R_i_k_.size(), D_R_i_k_quat_.size());
}

//------------------------------------------------------------------------------
Quaternion QuaternionPreIntegrationState::integrateFirstOrderFwd(
    Quaternion q,
    Vector3 w_i,
    FloatType dt)
{
  return q * Quaternion(Vector3(w_i * dt));
}

//------------------------------------------------------------------------------
Quaternion QuaternionPreIntegrationState::integrateFirstOrderMid(
    Quaternion q,
    Vector3 w_i,
    Vector3 w_i_1,
    FloatType dt)
{
  return q * Quaternion(Vector3((w_i + w_i_1) * 0.5 * dt));
}

//------------------------------------------------------------------------------
std::pair<Quaternion, Matrix3> QuaternionPreIntegrationState::integrateRK(
    Quaternion q,
    Vector3 w_i,
    Vector3 w_i_1,
    FloatType dt,
    uint32_t order)
{

  auto omega = [](Vector3 w) -> Matrix4 {
    Matrix4 Omega;
    Omega << 0,    -w[0], -w[1], -w[2],
             w[0],    0,   w[2], -w[1],
             w[1], -w[2],     0,  w[0],
             w[2],  w[1], -w[0],     0;
    return Omega;
  };

  auto stateDerivative = [&omega](Vector3 w, Vector4 q) -> Vector4
  {
    return 0.5 * omega(w) * q;
  };

  // get phi_i+1 given phi_i and w_i
  auto covarTransitionDerivative = [](Vector3 w, Matrix3 phi) -> Matrix3
  {
    // A is the continuous-time state update: x_dot = A(t) * x(t)
    Matrix3 A = -skewSymmetric(w);

    return A * phi;
  };

  if (order == 3)
  {
    Vector3 w_k1 = w_i;
    Vector3 w_k2 = w_i + (w_i_1 - w_i) * 0.5; // midpoint interpolation
    Vector3 w_k3 = w_k2;

    Vector4 q_v = q.vector();

    Vector4 q_1_dot = stateDerivative(w_k1, q_v);
    Vector4 q_v_1 = q_v + 0.5 * dt * q_1_dot;

    Vector4 q_2_dot = stateDerivative(w_k2, q_v_1);
    Vector4 q_v_2 = q_v + dt * (-1 * q_1_dot + 2 * q_2_dot);

    Vector4 q_3_dot = stateDerivative(w_k3, q_v_2);

    Vector4 q_i_1_v = q_v + dt * (q_1_dot + 4 * q_2_dot + q_3_dot) * 1.0 / 6.0;
    q_i_1_v.normalize();

    ///////////////
    //! Covariance
    Matrix3 phi = Matrix3::Identity();
    // Use a runge-kutta approximation of the state-transition matrix phi
    Matrix3 phi_1_dot = covarTransitionDerivative(w_k1, phi);
    Matrix3 phi_1 = phi + 0.5 * dt * phi_1_dot;

    Matrix3 phi_2_dot = covarTransitionDerivative(w_k2, phi_1);
    Matrix3 phi_2 = phi + dt * (-1 * phi_1_dot + 2 * phi_2_dot);

    Matrix3 phi_3_dot = covarTransitionDerivative(w_k3, phi_2);
    Matrix3 phi_i_1 = Matrix3::Identity()
                      + dt * (phi_1_dot + 4 * phi_2_dot + phi_3_dot) * 1.0 / 6.0;

    return std::make_pair<Quaternion, Matrix3>(
          Quaternion(q_i_1_v[0], q_i_1_v[1], q_i_1_v[2], q_i_1_v[3]),
          Matrix3(phi_i_1));
  }
  else if (order == 4)
  {
    Vector3 w_k1 = w_i;
    Vector3 w_k2 = w_i + (w_i_1 - w_i) * 0.5; // midpoint interpolation
    Vector3 w_k3 = w_k2;
    Vector3 w_k4 = w_i_1;

    Vector4 q_v = q.vector();

    Vector4 q_1_dot = stateDerivative(w_k1, q_v);
    Vector4 q_v_1 = q_v + 0.5 * dt * q_1_dot;

    Vector4 q_2_dot = stateDerivative(w_k2, q_v_1);
    Vector4 q_v_2 = q_v + 0.5 * dt * q_2_dot;

    Vector4 q_3_dot = stateDerivative(w_k3, q_v_2);
    Vector4 q_v_3 = q_v + dt * q_3_dot;

    Vector4 q_4_dot = stateDerivative(w_k4, q_v_3);

    Vector4 q_i_1_v = q_v + dt * (q_1_dot + 2 * q_2_dot + 2 * q_3_dot + q_4_dot) * 1.0/6.0;
    q_i_1_v.normalize();

    ///////////////
    //! Covariance
    Matrix3 phi = Matrix3::Identity();
    // Use a runge-kutta approximation of the state-transition matrix phi
    Matrix3 phi_1_dot = covarTransitionDerivative(w_k1, phi);
    Matrix3 phi_1 = phi + 0.5 * dt * phi_1_dot;

    Matrix3 phi_2_dot = covarTransitionDerivative(w_k2, phi_1);
    Matrix3 phi_2 = phi + 0.5 * dt * phi_2_dot;

    Matrix3 phi_3_dot = covarTransitionDerivative(w_k3, phi_2);
    Matrix3 phi_3 = phi + dt * phi_3_dot;

    Matrix3 phi_4_dot = covarTransitionDerivative(w_k4, phi_3);
    Matrix3 phi_i_1 = Matrix3::Identity()
                      + dt * 1.0 / 6.0 *
                      (phi_1_dot + 2 * phi_2_dot + 2 * phi_3_dot + phi_4_dot);

    return std::make_pair<Quaternion, Matrix3>(
          Quaternion(q_i_1_v[0], q_i_1_v[1], q_i_1_v[2], q_i_1_v[3]),
          Matrix3(phi_i_1));
  }

  throw std::runtime_error("Unsupported Runge-Kutta Integration Order");

  return std::make_pair<Quaternion, Matrix3>(Quaternion(), Matrix3::Identity());
}

//------------------------------------------------------------------------------
Quaternion QuaternionPreIntegrationState::integrateCrouchGrossman(
    Quaternion q,
    Vector3 w_i,
    Vector3 w_i_1,
    FloatType dt,
    uint32_t order)
{
  auto exp = [](FloatType coeff, FloatType dt, Vector3 w) -> Quaternion {
    return Quaternion::exp(dt * coeff * w);
  };

  if (order == 3)
  {
    Vector3 K_1 = w_i;
    Vector3 K_2 = w_i + (w_i_1 - w_i) * 3.0 / 4.0;
    Vector3 K_3 = w_i + (w_i_1 - w_i) * 17.0 / 24.0;

    return q *
        exp(13.0 / 51.0, dt, K_1) *
        exp(-2.0 / 3.0, dt, K_2) *
        exp(24.0 / 17.0, dt, K_3);
  }
  else if (order == 4)
  {
    Vector3 K_1 = w_i;
    Vector3 K_2 = w_i + (w_i_1 - w_i) * 0.8177227988124852;
    Vector3 K_3 = w_i + (w_i_1 - w_i) * 0.3859740639032449;
    Vector3 K_4 = w_i + (w_i_1 - w_i) * 0.3242290522866937;
    Vector3 K_5 = w_i + (w_i_1 - w_i) * 0.8768903263420429;

    return q *
        exp(0.1370831520630755, dt, K_1) *
        exp(-0.0183698531564020, dt, K_2) *
        exp(0.7397813985370780, dt, K_3) *
        exp(-0.1907142565505889, dt, K_4) *
        exp(0.3322195591068374, dt, K_5);
  }

  throw std::runtime_error("Unsupported Crouch Grossman Integration Order");

  return Quaternion();
}

//------------------------------------------------------------------------------
void QuaternionPreIntegrationState::doPushFirstOrderFwd(
    times_container_t stamps,
    measurements_container_t measurements)
{
  // Integrate measurements between frames.
  for (int i = 0; i < measurements.cols() - 1; ++i)
  {
    timers_[IntegrationTimer::integrate].start();

    FloatType dt = stamps[i+1] - stamps[i];

    // Reset to 0 at every step:
    if (i == 0)
    {
      D_R_i_k_quat_.push_back(Quaternion());
      if (compute_absolutes_)
      {
        R_i_k_quat_.push_back(integrateFirstOrderFwd(
                                R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
                                dt));
      }

      covariance_i_k_.push_back(Matrix3::Zero());
    }
    else
    {
      // Integrate
      D_R_i_k_quat_.push_back(integrateFirstOrderFwd(
                                D_R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
                                dt));
      D_R_i_k_quat_.back().normalize();

      if (compute_absolutes_)
      {
        R_i_k_quat_.push_back(integrateFirstOrderFwd(
                                R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
                                dt));
        R_i_k_quat_.back().normalize();
      }

      // Covariance Prediction (FWD Integrated)
      Matrix3 gyro_noise_covariance_d = gyro_noise_covariance_ / dt;
      Matrix3 D_R = Quaternion(Vector3(measurements.col(i).tail<3>(3) * dt)).
                    getRotationMatrix();
      covariance_i_k_.push_back(
            D_R.transpose() * covariance_i_k_.back() * D_R
            + gyro_noise_covariance_d * dt * dt);
    }

    times_raw_.push_back(stamps[i]);

    timers_[IntegrationTimer::integrate].stop();
  }
}

//------------------------------------------------------------------------------
void QuaternionPreIntegrationState::doPushFirstOrderMid(
    times_container_t stamps,
    measurements_container_t measurements)
{
  size_t previous_items = D_R_i_k_quat_.size();

  // Integrate measurements between frames.
  for (int i = 0; i < measurements.cols() - 1; ++i)
  {
    timers_[IntegrationTimer::integrate].start();

    FloatType dt = stamps[i+1] - stamps[i];

    // Reset to 0 at every step:
    if (i == 0)
    {
      D_R_i_k_quat_.push_back(Quaternion());

      if(compute_absolutes_)
      {
        R_i_k_quat_.push_back(integrateFirstOrderMid(
                                R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
                                measurements.col(i + 1).tail<3>(3),
                                dt));
      }
      covariance_i_k_.push_back(Matrix3::Zero());
    }
    else
    {
      // Integrate
      D_R_i_k_quat_.push_back(integrateFirstOrderMid(
                                D_R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
                                measurements.col(i + 1).tail<3>(3),
                                dt));
      D_R_i_k_quat_.back().normalize();
      if(compute_absolutes_)
      {
        R_i_k_quat_.push_back(integrateFirstOrderMid(
                                R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
                                measurements.col(i + 1).tail<3>(3),
                                dt));
        R_i_k_quat_.back().normalize();
      }

      // Covariance Prediction (MID Integrated)
      Matrix3 gyro_noise_covariance_d = gyro_noise_covariance_ / dt;
      Vector3 mid_point = (measurements.col(i).tail<3>(3) +
                           measurements.col(i + 1).tail<3>(3)) * 0.5;
      Matrix3 D_R = Quaternion(Vector3(mid_point * dt)).getRotationMatrix();

      covariance_i_k_.push_back(
            D_R.transpose() * covariance_i_k_.back() * D_R
            + gyro_noise_covariance_d * dt * dt);
    }

    times_raw_.push_back(stamps[i]);

    timers_[IntegrationTimer::integrate].stop();
  }
}

//------------------------------------------------------------------------------
void QuaternionPreIntegrationState::doPushRK(
    times_container_t stamps,
    measurements_container_t measurements,
    uint32_t order)
{
  // Integrate measurements between frames.
  for (int i = 0; i < measurements.cols() - 1; ++i)
  {
    timers_[IntegrationTimer::integrate].start();

    FloatType dt = stamps[i+1] - stamps[i];

    // Reset to 0 at every step:
    if (i == 0)
    {
      D_R_i_k_quat_.push_back(Quaternion());

      Quaternion q_i_1;
      std::tie(q_i_1, std::ignore) = integrateRK(
                                       R_i_k_quat_.back(),
                                       measurements.col(i).tail<3>(3),
                                       measurements.col(i+1).tail<3>(3),
                                       dt,
                                       order);
      if (compute_absolutes_)
      {
        R_i_k_quat_.push_back(q_i_1);
      }
      covariance_i_k_.push_back(Matrix3::Zero());
    }
    else
    {
      // Integrate
      Quaternion q_i_1;
      Matrix3 S_i_1;

      std::tie(q_i_1, S_i_1) = integrateRK(
                                 D_R_i_k_quat_.back(),
                                 measurements.col(i).tail<3>(3),
                                 measurements.col(i + 1).tail<3>(3),
                                 dt,
                                 order);

      D_R_i_k_quat_.push_back(q_i_1);
      D_R_i_k_quat_.back().normalize();

      if (compute_absolutes_)
      {
        Quaternion q_i_1_global;
        std::tie(q_i_1_global, std::ignore) = integrateRK(
                                                R_i_k_quat_.back(),
                                                measurements.col(i).tail<3>(3),
                                                measurements.col(i + 1).tail<3>(3),
                                                dt,
                                                order);
        R_i_k_quat_.push_back(q_i_1_global);
        R_i_k_quat_.back().normalize();
      }

      // Covariance Prediction
      Matrix3 gyro_noise_covariance_d = gyro_noise_covariance_ / dt;

      covariance_i_k_.push_back(
            S_i_1.transpose() * covariance_i_k_.back() * S_i_1
            + gyro_noise_covariance_d * dt * dt);
    }

    times_raw_.push_back(stamps[i]);

    timers_[IntegrationTimer::integrate].stop();
  }
}

//------------------------------------------------------------------------------
void QuaternionPreIntegrationState::doPushCrouchGrossman(
    times_container_t stamps,
    measurements_container_t measurements,
    uint32_t order)
{
  // Integrate measurements between frames.
  for (int i = 0; i < measurements.cols() - 1; ++i)
  {
    timers_[IntegrationTimer::integrate].start();

    FloatType dt = stamps[i+1] - stamps[i];

    // Reset to 0 at every step:
    if (i == 0)
    {
      D_R_i_k_quat_.push_back(Quaternion());
      if (compute_absolutes_)
      {
        R_i_k_quat_.push_back(integrateCrouchGrossman(
                                R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
                                measurements.col(i+1).tail<3>(3),
                                dt,
                                order));
      }

      covariance_i_k_.push_back(Matrix3::Zero());
    }
    else
    {
      // Integrate
      D_R_i_k_quat_.push_back(integrateCrouchGrossman(
                                D_R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
                                measurements.col(i + 1).tail<3>(3),
                                dt,
                                order));
      D_R_i_k_quat_.back().normalize();

      if (compute_absolutes_)
      {
        R_i_k_quat_.push_back(integrateCrouchGrossman(
                                R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
                                measurements.col(i + 1).tail<3>(3),
                                dt,
                                order));
        R_i_k_quat_.back().normalize();
      }

      // Covariance Prediction (FWD Integrated)
      // @todo: implement native CG3/CG4 propagation
      Matrix3 gyro_noise_covariance_d = gyro_noise_covariance_ / dt;
      Matrix3 D_R = Quaternion(Vector3((measurements.col(i).tail<3>(3) +
                                        measurements.col(i + 1).tail<3>(3)) * 0.5 * dt)).
                    getRotationMatrix();

      covariance_i_k_.push_back(
            D_R.transpose() * covariance_i_k_.back() * D_R
            + gyro_noise_covariance_d * dt * dt);
    }

    times_raw_.push_back(stamps[i]);

    timers_[IntegrationTimer::integrate].stop();
  }
}

} // namespace ze
