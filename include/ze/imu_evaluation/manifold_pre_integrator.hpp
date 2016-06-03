#pragma once

#include <ze/common/types.h>
#include <ze/imu_evaluation/pre_integrator_base.hpp>
#include <ze/common/transformation.h>
#include <ze/imu_evaluation/scenario_runner.hpp>

namespace ze {

//! A class that represents all manifold pre-integrated orientations and
//! the corresponding propagated covariance.
class ManifoldPreIntegrationState : public PreIntegrator
{
public:
  ManifoldPreIntegrationState(Matrix3 gyro_noise_covariance)
    : PreIntegrator(gyro_noise_covariance)
  {}

  void pushD_R(times_container_t stamps,
               measurements_container_t measurements)
  {
    Matrix3 D_R = Matrix3::Identity();
    // integrate measurements between frames
    for (size_t i = 0; i < measurements.size(); ++i)
    {
      //! @todo revert loop for better efficiency D_R *= ...
      D_R = Quaternion::exp(
              measurements[i] * (stamps[i+1] - stamps[i])).getRotationMatrix()
          * D_R;
    }

    D_R_.push_back(D_R);
    times_.push_back(*stamps.rbegin());

    propagate_covariance();
  }

  void propagate_covariance()
  {
    if (covariances_.size() == 0)
    {
      covariances_.push_back(Matrix3::Zero());

      return;
    }

    Vector3 psi = ((Quaternion(*D_R_.rbegin())).log());
    FloatType norm = psi.norm();
    FloatType norm_sqr = norm*norm;
    Matrix3 J_r = Matrix3::Identity()
                  - (1 - cos(norm)) / (norm_sqr) * skewSymmetric(psi)
                  + (norm - sin(norm)) / (norm_sqr * norm)
                  * skewSymmetric(psi) * skewSymmetric(psi);

    FloatType dt = *(times_.end() - 1) - *(times_.end() - 2);

    covariances_.push_back(
          (*D_R_.rbegin()) * (*covariances_.rbegin()) * D_R_.rbegin()->transpose()
          + J_r * gyro_noise_covariance_ * dt * J_r.transpose());
  }
};

} // namespace ze
