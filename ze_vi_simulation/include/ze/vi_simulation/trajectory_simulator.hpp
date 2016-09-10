// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

// The below code and structure is heavily inspired by gtsam's
// navigation/Scenario.h and ScenarioRunner.h

#include <ze/common/macros.hpp>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

//! Represents a test scenario that describes a trajectory.
class TrajectorySimulator
{
public:
  ZE_POINTER_TYPEDEFS(TrajectorySimulator);

  //! Get the pose at a given time.
  virtual Transformation T_W_B(real_t t) const = 0;

  //! Get the rotational velocity in the body frame.
  virtual Vector3 angularVelocity_B(real_t t) const = 0;

  //! Get the velocity in the world frame.
  virtual Vector3 velocity_W(real_t t) const = 0;

  //! Get the acceleration in the world frame (without gravity).
  virtual Vector3 acceleration_W(real_t t) const = 0;

  //! Start time of the scenario
  virtual real_t start() const = 0;

  //! End time of the scenario
  virtual real_t end() const = 0;

  //! Get the orientation in the world frame.
  Quaternion R_W_B(real_t t) const
  {
    return T_W_B(t).getRotation();
  }

  //! The linear velocity in the body frame.
  Vector3 velocity_B(real_t t) const
  {
    const Quaternion Rwb = R_W_B(t);
    return Rwb.inverse().rotate(velocity_W(t));
  }

  //! The linear acceleration in the body frame (without gravity).
  Vector3 acceleration_B(real_t t) const
  {
    const Quaternion Rwb = R_W_B(t);
    return Rwb.inverse().rotate(acceleration_W(t));
  }
};

//! A scenario that is based upon a bspline fitted trajectory.
class SplineTrajectorySimulator : public TrajectorySimulator
{
public:
  SplineTrajectorySimulator(const std::shared_ptr<BSplinePoseMinimalRotationVector>& bs)
    : bs_(bs)
  {}

  //! Get pose at time t.
  virtual Transformation T_W_B(real_t t) const override
  {
    return Transformation(bs_->transformation(t));
  }

  //! Get gyro measurement at time t (B_w_WB).
  virtual Vector3 angularVelocity_B(real_t t) const override
  {
    return bs_->angularVelocityBodyFrame(t);
  }

  //! Get velocity in world coorinates at time t.
  virtual  Vector3 velocity_W(real_t t) const override
  {
    return bs_->linearVelocity(t);
  }

  //! Get acceleration in world coordinates at time t (without gravity).
  virtual Vector3 acceleration_W(real_t t) const override
  {
    return bs_->linearAcceleration(t);
  }

  //! Get start-time of trajectory.
  virtual real_t start() const override
  {
    return bs_->t_min();
  }

  //! Get end time of trajectory.
  virtual real_t end() const override
  {
    return bs_->t_max();
  }

private:
  const std::shared_ptr<BSplinePoseMinimalRotationVector> bs_;
};

} // namespace ze
