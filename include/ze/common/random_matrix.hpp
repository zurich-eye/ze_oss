#pragma once

#include <ze/common/random.hpp>
#include <ze/common/types.h>
#include <ze/common/macros.h>

namespace ze {

//------------------------------------------------------------------------------
//! A sampler for uncorrelated noise vectors.
template<size_t DIM>
class RandomVectorSampler
{
public:
  ZE_POINTER_TYPEDEFS(RandomVectorSampler);

  typedef Eigen::Matrix<FloatType, DIM, DIM> covariance_matrix_t;
  typedef Eigen::Matrix<FloatType, DIM, 1> covariance_vector_t;
  typedef Eigen::Matrix<FloatType, DIM, 1> sigma_vector_t;
  typedef Eigen::Matrix<FloatType, DIM, 1> noise_vector_t;

  //! Get a noise sample.
  noise_vector_t sample()
  {
    noise_vector_t noise;
    for (size_t i = 0; i < DIM; ++i)
    {
      // The gaussian takes a standard deviation as input.
      noise(i) = sampleFromNormalDistribution<FloatType>(deterministic_, 0.0, sigma_(i));
    }
    return noise;
  }

  static Ptr sigmas(const sigma_vector_t& sigmas, bool deterministic = false)
  {
    Ptr noise(new RandomVectorSampler(deterministic));
    noise->sigma_ = sigmas;
    return noise;
  }

  static Ptr variances(const covariance_vector_t& variances, bool deterministic = false)
  {
    Ptr noise(new RandomVectorSampler(deterministic));
    noise->sigma_ = variances.cwiseSqrt();
    return noise;
  }

protected:
  RandomVectorSampler(bool deteterministic)
    : deterministic_(deteterministic)
  {}

private:
  const bool deterministic_;
  sigma_vector_t sigma_;
};

//------------------------------------------------------------------------------
template<int rows, int cols>
Eigen::Matrix<FloatType, rows, cols>
randomMatrixUniformDistributed(
    bool deterministic = false,
    FloatType from = 0.0,
    FloatType to   = 1.0)
{
  DEBUG_CHECK_GT(rows, 0);
  DEBUG_CHECK_GT(cols, 0);
  Eigen::Matrix<FloatType, rows, cols> m;
  for (int x = 0; x < cols; ++x)
  {
    for (int y = 0; y < rows; ++y)
    {
      m(y,x) = sampleFromUniformRealDistribution(deterministic, from, to);
    }
  }
  return m;
}

//------------------------------------------------------------------------------
template<int size>
Eigen::Matrix<FloatType, size, 1>
randomVectorUniformDistributed(
    bool deterministic = false,
    FloatType from = 0.0,
    FloatType to   = 1.0)
{
  return randomMatrixUniformDistributed<size, 1>(deterministic, from, to);
}

//------------------------------------------------------------------------------
template<int rows, int cols>
Eigen::Matrix<FloatType, rows, cols>
randomMatrixNormalDistributed(
    bool deterministic = false,
    FloatType mean  = 0.0,
    FloatType sigma = 1.0)
{
  DEBUG_CHECK_GT(rows, 0);
  DEBUG_CHECK_GT(cols, 0);
  Eigen::Matrix<FloatType, rows, cols> m;
  for (int x = 0; x < cols; ++x)
  {
    for (int y = 0; y < rows; ++y)
    {
      m(y,x) = sampleFromNormalDistribution(deterministic, mean, sigma);
    }
  }
  return m;
}

//------------------------------------------------------------------------------
template<int size>
Eigen::Matrix<FloatType, size, 1>
randomVectorNormalDistributed(
    bool deterministic = false,
    FloatType mean  = 0.0,
    FloatType sigma = 1.0)
{
  return randomMatrixNormalDistributed<size, 1>(deterministic, mean, sigma);
}

} // namespace ze
