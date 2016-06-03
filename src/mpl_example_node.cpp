#include <ze/visualization/matplotlibcpp.hpp>
#include <ze/common/types.h>

namespace plt = matplotlibcpp;
int main()
{
  using namespace ze;

  // Simple:
  std::vector<double> v({1, 2, 3, 4});
  plt::plot(v);
  plt::show();

  // Eigen Vector Types:
  VectorX times(100);
  times.setLinSpaced(100, 0, 20);
  VectorX points(100);
  points.setRandom();

  plt::plot(times, points);
  plt::show();

  plt::named_plot("A Name", times, points);
  plt::show();

}
