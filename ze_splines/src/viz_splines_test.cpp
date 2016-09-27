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

#include <ros/ros.h>
#include <ze/common/logging.hpp>
#include <gflags/gflags.h>

#include <ze/visualization/viz_ros.hpp>
#include <ze/splines/viz_splines.hpp>
#include <ze/splines/bspline.hpp>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/common/types.hpp>

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Visualizer internally initializes ROS and creates a node handle.
  std::shared_ptr<ze::VisualizerRos> rv = std::make_shared<ze::VisualizerRos>();
  ze::SplinesVisualizer sv(rv);

  // Create a random 3d spline.
  ze::BSpline bs(3);

  ze::VectorX times = ze::VectorX::LinSpaced(500, 0, 100);
  ze::MatrixX points;
  points.resize(3, times.size());
  points.setRandom();

  bs.initSpline3(times, points, 100, 1e-5);
  sv.displaySplineTrajectory(bs, "spline", 0, ze::Colors::Green);

  // Create a random pose spline.
  ze::BSplinePoseMinimalRotationVector pbs(3);
  ze::MatrixX poses;
  poses.resize(6, times.size());
  poses.setRandom();
  poses.topRows(3) *= 10;

  pbs.initPoseSpline3(times, poses, 100, 1e-5);

  sv.displaySplineTrajectory(pbs, "poses", 0);

  // 2d Plots
  sv.plotSpline(bs, 0);

  // plot a pose spline as 6 dimensions
  sv.plotSpline(pbs);
}
