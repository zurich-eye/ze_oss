// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
