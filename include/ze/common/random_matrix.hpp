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

  static Ptr sigmas(const covariance_vector_t& sigmas, bool deterministic = false)
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
  covariance_vector_t sigma_;
};

//------------------------------------------------------------------------------
template<int size>
Eigen::Matrix<FloatType, size, 1>
randomVectorUniformlyDistributed(
    bool deterministic = false,
    FloatType from = 0.0,
    FloatType to   = 1.0)
{
  DEBUG_CHECK_GT(size, 0);
  Eigen::Matrix<FloatType, size, 1> v;
  for (int i = 0; i < size; ++i)
  {
    v(i) = sampleFromUniformRealDistribution(deterministic, from, to);
  }
  return v;
}

} // namespace ze
