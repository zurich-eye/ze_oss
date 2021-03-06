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

#include <ze/matplotlib/matplotlibcpp.hpp>

int main()
{
  using namespace ze;

  // Simple:
  std::vector<real_t> v({1, 2, 3, 4});
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
  std::vector<real_t> w({4, 3, 2, 1});
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
