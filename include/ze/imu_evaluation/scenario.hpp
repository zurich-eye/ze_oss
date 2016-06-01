#pragma once

// The below code and structure is heavily inspired by gtsam's
// navigation/Scenario.h and ScenarioRunner.h

#include <ze/common/macros.h>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/common/transformation.h>

namespace ze {

//! Represents a test scenario that describes a trajectory.
class Scenario
{
public:
  ZE_POINTER_TYPEDEFS(Scenario);

  //! Get the pose at a given time.
  virtual Transformation T_W_B(FloatType t) const = 0;

  //! Get the rotational velocity in the body frame.
  virtual Vector3 angular_velocity_B(FloatType t) const = 0;

  //! Get the velocity in the inertial frame.
  virtual Vector3 velocity_W(FloatType t) const = 0;

  //! Get the acceleration in the inertial frame.
  virtual Vector3 acceleration_W(FloatType t) const = 0;

  //! Get the orientation in the inertial frame.
  Quaternion R_W_B(FloatType t) const
  {
    return T_W_B(t).getRotation();
  }

  //! The linear velocity in the body frame.
  Vector3 velocity_B(FloatType t) const
  {
    const Quaternion Rwb = R_W_B(t);
    return Rwb.inverse().rotate(velocity_W(t));
  }

  //! The linear acceleration in the body frame.
  Vector3 acceleration_B(FloatType t) const
  {
    const Quaternion Rwb = R_W_B(t);
    return Rwb.inverse().rotate(acceleration_W(t));
  }
};

//! A scenario that is based upon a bspline fitted trajectory.
class SplineScenario: public Scenario
{
public:
  SplineScenario(const BSplinePoseMinimalRotationVector& bs)
    : bs_(bs)
  {}

  Transformation T_W_B(FloatType t) const
  {
    return Transformation(bs_.transformation(t));
  }

  Vector3 angular_velocity_B(FloatType t) const
  {
    return bs_.angularVelocityBodyFrame(t);
  }

  Vector3 velocity_W(FloatType t) const
  {
    return bs_.linearVelocity(t);
  }

  Vector3 acceleration_W(FloatType t) const
  {
    return bs_.linearAcceleration(t);
  }

private:
  const BSplinePoseMinimalRotationVector& bs_;
};

} // namespace ze
