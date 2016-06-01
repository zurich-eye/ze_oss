#pragma once

// The below code and structure is heavily inspired by gtsam's
// navigation/Scenario.h and ScenarioRunner.h

#include <ze/common/macros.h>
#include <ze/splines/bspline_pose_minimal.hpp>

namespace ze {

//! Represents a test scenario that describes a trajectory.
class Scenario
{
public:
  ZE_POINTER_TYPEDEFS(Scenario);

  //! Get the pose at a given time.
  virtual Matrix4 pose(double t) const = 0;

  //! Get the rotational velocity in the body frame.
  virtual Vector3 angular_velocity_body(double t) const = 0;

  //! Get the velocity in the inertial frame.
  virtual Vector3 velocity(double t) const = 0;

  //! Get the acceleration in the inertial frame.
  virtual Vector3 acceleration(double t) const = 0;

  //! Get the orientation in the inertial frame.
  Matrix3 orientation(double t) const
  {
    return pose(t).block<3, 3>(0, 0);
  }

  //! The linear velocity in the body frame.
  Vector3 velocity_body(double t) const
  {
    const Matrix3 Rib = orientation(t);
    return Rib.transpose() * velocity(t);
  }

  //! The linear acceleration in the body frame.
  Vector3 acceleration_body(double t) const
  {
    const Matrix3 Rib = orientation(t);
    return Rib.transpose() * acceleration(t);
  }
};

//! A scenario that is based upon a bspline fitted trajectory.
class SplineScenario: public Scenario
{
public:
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
