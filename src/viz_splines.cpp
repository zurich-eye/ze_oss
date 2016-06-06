#include <ze/splines/viz_splines.hpp>

namespace ze {

SplinesVisualizer::SplinesVisualizer(const std::shared_ptr<Visualizer>& viz)
  : viz_(viz)
{}

void SplinesVisualizer::plotSpline(const BSpline& bs,
                                   const size_t dimension,
                                   FloatType step_size)
{
  FloatType start = bs.t_min();
  FloatType end = bs.t_max();
  size_t samples = (end - start) / step_size;
  size_t spline_dimension = bs.dimension();
  CHECK_LE(dimension, spline_dimension);

  ze::VectorX points(samples);
  ze::VectorX times(samples);

  for (size_t i = 0; i < samples; ++i)
  {
    points(i) = (bs.eval(start + i * step_size))(dimension);
    times(i) = start + i * step_size;
  }

  matplotlibcpp::plot(times, points);
  matplotlibcpp::show();
}

void SplinesVisualizer::displaySpline(const BSpline& bs,
                                      const std::string& topic,
                                      const size_t id,
                                      const Color& color,
                                      FloatType step_size)
{
  CHECK_LE(bs.dimension(), 3) << "For splines with more than 3 dimensions specify"
                          << "which dimenions to draw.";

  switch (bs.dimension())
  {
    case 1:
      return displaySpline(bs, topic, id, color, {0}, step_size);
    case 2:
      return displaySpline(bs, topic, id, color, {0, 1}, step_size);
    case 3:
      return displaySpline(bs, topic, id, color, {0, 1, 2}, step_size);
  }

}

void SplinesVisualizer::displaySpline(const BSpline& bs,
                                      const std::string& topic,
                                      const size_t id,
                                      const Color& color,
                                      const std::vector<size_t>& draw_dimensions,
                                      FloatType step_size)
{
  FloatType start = bs.t_min();
  FloatType end = bs.t_max();
  size_t samples = (end - start) / step_size;
  size_t dimension = bs.dimension();

  ze::Positions points(3, samples);
  points.setZero();

  //! @todo: this is terribly inefficient.
  for (size_t i = 0; i < samples; ++i)
  {
    VectorX v = bs.eval(start + i * step_size);
    for (size_t j = 0; j < draw_dimensions.size(); ++j)
    {
      points(j, i) = v(draw_dimensions[j]);
    }
  }

  viz_->drawPoints(topic, id, points, color);
}

} // namespace ze
