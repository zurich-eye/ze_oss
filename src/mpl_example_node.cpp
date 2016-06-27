#include <ze/matplotlib/matplotlibcpp.hpp>

int main()
{
  using namespace ze;

  // Simple:
  std::vector<FloatType> v({1, 2, 3, 4});
  plt::plot(v);
  plt::show();

  // Eigen Vector Types:
  VectorX times(100);
  times.setLinSpaced(100, 0, 20);
  VectorX points(100);
  points.setRandom();

  plt::plot(times, points);
  plt::show();

  plt::labelPlot("A Name", times, points);
  plt::show();

  // enable interactive mode as of now (only tests if it doesn't crash)
  plt::ion();

  // subplots
  plt::subplot(3, 1, 1);
  plt::plot(v);
  plt::subplot(3, 1, 2);
  plt::plot(v);
  plt::subplot(3, 1, 3);
  plt::plot(v);
  plt::show(false);

  plt::figure();

  // plot multiple curves in a single graph
  std::vector<FloatType> w({4, 3, 2, 1});
  plt::plot(v, "x");
  plt::plot(w, "o");
  plt::show();

  // Histogram
  plt::hist(points, 3);
  plt::show();

  // Row vectors
  MatrixX matrix(2, 100);
  matrix.setRandom();
  plt::plot(matrix.row(0), matrix.row(1));
  plt::show();

  // BoxPlot
  MatrixX data(2, 100);
  data.setRandom();
  plt::figure();
  std::vector<std::string> labels = {"A", "B"};
  plt::boxplot(data, labels);
  plt::show();


  // BoxPlot
  data.setRandom();
  plt::figure();
  plt::boxplot(data, {"A", "B"});
  plt::show();

  // Boxplot unlabelled
  data.setRandom();
  plt::figure();
  plt::boxplot(data);
  plt::show();

}
