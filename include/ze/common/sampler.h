#pragma once

#include <ze/common/sample.h>
#include <ze/common/types.h>
#include <ze/common/macros.h>

namespace ze {

//! a sampler for uncorrelated noise vectors
template<size_t DIM>
class GaussianSampler
{
public:
  ZE_POINTER_TYPEDEFS(GaussianSampler);

  typedef Eigen::Matrix<FloatType, DIM, DIM> covariance_matrix_t;
  typedef Eigen::Matrix<FloatType, DIM, 1> covariance_vector_t;
  typedef Eigen::Matrix<FloatType, DIM, 1> noise_vector_t;

  //! given a diagonal covariance matrix
  GaussianSampler(const covariance_matrix_t& Sigma) : Sigma_(Sigma.diagonal())
  {
    // ensure a diagonal matrix
    for (size_t i = 0; i < DIM; ++i)
    {
      for (size_t j = 0; j < DIM; ++j)
      {
        if (i != j)
        {
          CHECK_EQ(0, Sigma_(i, j));
        }
      }
    }
  }

  //! given a covariance vector
  GaussianSampler(const covariance_vector_t& Sigma_vector)
    : Sigma_(Sigma_vector)
  {
  }

  //! get a noise sample
  noise_vector_t sample()
  {
    noise_vector_t noise;
    for (size_t i = 0; i < DIM; ++i)
    {
      noise(i) = Sample::gaussian(Sigma_(i));
    }
    return noise;
  }

private:
  covariance_vector_t Sigma_;
};

} // namespace ze
