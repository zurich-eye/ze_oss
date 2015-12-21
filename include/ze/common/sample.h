#pragma once

#include <random>
#include <chrono>
#include <Eigen/Core>

namespace ze {

class Sample
{
public:
  static void setTimeBasedSeed();

  static int uniform(int from, int to);
  static double uniform();
  static double gaussian(double sigma);
  static Eigen::Vector3d randomDirection3D();
  static Eigen::Vector2d randomDirection2D();

  static std::ranlux24 gen_real;
  static std::mt19937 gen_int;
};

} // namespace ze
