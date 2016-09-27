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

#include <memory>

#include <ze/common/types.hpp>
#include <ze/splines/bspline.hpp>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/visualization/viz_common.hpp>

namespace ze {

// fwd
class Visualizer;

class SplinesVisualizer
{
public:
  SplinesVisualizer(const std::shared_ptr<Visualizer>& viz);

  //! Plot a single dimension of a spline.
  void plotSpline(
      const BSpline& bs,
      const size_t dimension,
      real_t step_size = 0.05);

  //! Display a spline (max 3 dimensions) as point curve in space.
  void displaySplineTrajectory(
      const BSpline& bs,
      const std::string& topic,
      const size_t id,
      const Color& color,
      real_t step_size = 0.05);

  //! Display a spline (max 3 dimensions) as point curve specifying the dimensions
  //! of the spline to plot.
  void displaySplineTrajectory(
      const BSpline& bs,
      const std::string& topic,
      const size_t id,
      const Color& color,
      const std::vector<size_t>& draw_dimensions,
      real_t step_size = 0.05);

  //! Display a pose spline as pose-curve.
  void displaySplineTrajectory(
      const BSplinePoseMinimalRotationVector& bs,
      const std::string& topic,
      const size_t id,
      real_t step_size = 0.05);

  //! Plot a pose spline as one graph for rotational and one graph
  //! for translational components.
  void plotSpline(
      const BSplinePoseMinimalRotationVector& bs,
      real_t step_size = 0.05);

private:
  std::shared_ptr<Visualizer> viz_;
};

} // namespace ze
