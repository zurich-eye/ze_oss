#pragma once

// The below code and structure is heavily inspired by gtsam's
// navigation/Scenario.h and ScenarioRunner.h

#include <ze/common/macros.h>
#include <ze/splines/bspline_pose_minimal.h>

namespace ze {

//! represents a test scenario with a given trajectory
class Scenario
{
public:
  ZE_POINTER_TYPEDEFS(Scenario);

  virtual Matrix4 pose(double t) const = 0;

  //! rotational velocity in body frame
  virtual Vector3 angular_velocity_body(double t) const = 0;

  //! velocity in the inertial frame
  virtual Vector3 velocity(double t) const = 0;

  //! acceleration in the inertial frame
  virtual Vector3 acceleration(double t) const = 0;

  //! orientation in inertial frame
  Matrix3 orientation(double t) const
  {
    return pose(t).block<3, 3>(0, 0);
  }

  //! linear velocity in body frame
  Vector3 velocity_body(double t) const
  {
    const Matrix3 Rbi = orientation(t);
    return Rbi.transpose() * velocity(t);
  }

  //! linear acceleration in body frame
  Vector3 acceleration_body(double t) const
  {
    const Matrix3 Rbi = orientation(t);
    return Rbi.transpose() * acceleration(t);
  }
};

//! a scenario that is based upon a bspline fitted trajectory
class SplineScenario: public Scenario
{
public:
  //! construct with an initialized pose spline
  SplineScenario(const BSplinePoseMinimalRotationVector& bs)
    : bs_(bs)
  {}

  Matrix4 pose(double t) const
  {
    return bs_.transformation(t);
  }

  Vector3 angular_velocity_body(double t) const
  {
    return bs_.angularVelocityBodyFrame(t);
  }

  Vector3 velocity(double t) const
  {
    return bs_.linearVelocity(t);
  }

  Vector3 acceleration(double t) const
  {
    return bs_.linearAcceleration(t);
  }

private:
  const BSplinePoseMinimalRotationVector& bs_;
};

} // namespace ze
