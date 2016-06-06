#pragma once

#include <ze/visualization/viz_interface.h>
#include <ze/splines/bspline.hpp>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/matplotlib/matplotlibcpp.hpp>

namespace ze {

class SplinesVisualizer
{
public:
  SplinesVisualizer(const std::shared_ptr<Visualizer>& viz);

  //! Plot a single dimension of a spline.
  void plotSpline(const BSpline& bs,
                  const size_t dimension,
                  FloatType step_size = 0.05);

  //! Display a spline (max 3 dimensions) as point curve in space.
  void displaySplineTrajectory(const BSpline& bs,
                               const std::string& topic,
                               const size_t id,
                               const Color& color,
                               FloatType step_size = 0.05);

  //! Display a spline (max 3 dimensions) as point curve specifying the dimensions
  //! of the spline to plot.
  void displaySplineTrajectory(const BSpline& bs,
                               const std::string& topic,
                               const size_t id,
                               const Color& color,
                               const std::vector<size_t>& draw_dimensions,
                               FloatType step_size = 0.05);

  //! Display a pose spline as pose-curve.
  template<class ROTATION>
  void displaySplineTrajectory(const BSplinePoseMinimal<ROTATION>& bs,
                               const std::string& topic,
                               const size_t id,
                               FloatType step_size = 0.05)
  {
    ze::TransformationVector poses;

    FloatType start = bs.t_min();
    FloatType end = bs.t_max();
    size_t samples = (end - start) / step_size;

    for (size_t i = 0; i <= samples; ++i)
    {
      Transformation T(bs.transformation(start + i * step_size));
      poses.push_back(T);
    }

    viz_->drawCoordinateFrames(topic, id, poses,  1);
  }

  //! Plot a pose spline as one graph for rotational and one graph
  //! for translational components.
  template<class ROTATION>
  void plotSpline(const BSplinePoseMinimal<ROTATION>& bs,
                  FloatType step_size = 0.05)
  {
    FloatType start = bs.t_min();
    FloatType end = bs.t_max();
    size_t samples = (end - start) / step_size;

    ze::MatrixX points(6, samples);
    ze::VectorX times(samples);

    for (size_t i = 0; i < samples; ++i)
    {
      points.col(i) = bs.eval(start + i * step_size);
      times(i) = start + i * step_size;
    }

    // translation
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::plot(times, points.row(0));
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::plot(times, points.row(1));
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::plot(times, points.row(2));
    matplotlibcpp::show();

    // rotation
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::plot(times, points.row(3));
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::plot(times, points.row(4));
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::plot(times, points.row(5));
    matplotlibcpp::show();
  }

private:
  std::shared_ptr<Visualizer> viz_;
};

} // namespace ze
