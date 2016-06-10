#pragma once

#include <memory>

#include <ze/common/types.h>
#include <ze/splines/bspline.hpp>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/visualization/viz_common.h>

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
      FloatType step_size = 0.05);

  //! Display a spline (max 3 dimensions) as point curve in space.
  void displaySplineTrajectory(
      const BSpline& bs,
      const std::string& topic,
      const size_t id,
      const Color& color,
      FloatType step_size = 0.05);

  //! Display a spline (max 3 dimensions) as point curve specifying the dimensions
  //! of the spline to plot.
  void displaySplineTrajectory(
      const BSpline& bs,
      const std::string& topic,
      const size_t id,
      const Color& color,
      const std::vector<size_t>& draw_dimensions,
      FloatType step_size = 0.05);

  //! Display a pose spline as pose-curve.
  void displaySplineTrajectory(
      const BSplinePoseMinimalRotationVector& bs,
      const std::string& topic,
      const size_t id,
      FloatType step_size = 0.05);

  //! Plot a pose spline as one graph for rotational and one graph
  //! for translational components.
  void plotSpline(
      const BSplinePoseMinimalRotationVector& bs,
      FloatType step_size = 0.05);

private:
  std::shared_ptr<Visualizer> viz_;
};

} // namespace ze
