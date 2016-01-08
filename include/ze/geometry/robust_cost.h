#pragma once

#include <vector>
#include <memory>
#include <glog/logging.h>
#include <Eigen/Core>

#include <ze/common/statistics.h>

namespace ze {

// -----------------------------------------------------------------------------
// Scale Estimators

//! Dummy scale estimator.
template <typename Scalar>
struct UnitScaleEstimator
{
  using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  static constexpr Scalar compute(const Vector& normed_errors)
  {
    return 1.0;
  }
};

//! Estimates scale by computing the median absolute deviation (MAD).
template <typename Scalar>
struct MADScaleEstimator
{
  using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  static Scalar compute(const Vector& normed_errors) const
  {
    std::pair<Scalar, bool> res = median(normed_errors);
    CHECK(res.second);
    return 1.48f * res.first; // 1.48f / 0.6745
  }
};

//! Estimates scale by computing the standard deviation.
template <typename Scalar>
struct NormalDistributionScaleEstimator
{
  using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  static Scalar compute(const Vector& normed_errors) const
  {
    const Scalar mean = normed_errors.sum() / normed_errors.size();
    Scalar variance = 0.0;
    for(int i = 0; i < normed_errors.size(); ++i)
    {
      variance += (normed_errors(i) - mean) * (normed_errors(i) - mean);
    }
    return std::sqrt(var); // return standard deviation
  }
};


// -----------------------------------------------------------------------------
// Scale Estimators
// Weight-Functions for M-Estimators
// http://research.microsoft.com/en-us/um/people/zhang/inria/publis/tutorial-estim/node24.html

class WeightFunction
{
public:
  WeightFunction() = default;
  virtual ~WeightFunction() = default;
  virtual float weight(const float& normed_errors) const = 0;
};
typedef std::shared_ptr<WeightFunction> WeightFunctionPtr;

class UnitWeightFunction : public WeightFunction
{
public:
  using WeightFunction::WeightFunction;
  virtual ~UnitWeightFunction() = default;
  virtual float weight(const float& normed_errors) const;
};

class TukeyWeightFunction : public WeightFunction
{
public:
  TukeyWeightFunction(const float b = 4.6851f);
  virtual ~TukeyWeightFunction() = default;
  virtual float weight(const float& normed_errors) const;
private:
  float b_square_;
};

class HuberWeightFunction : public WeightFunction
{
public:
  HuberWeightFunction(const float k = 1.345f);
  virtual ~HuberWeightFunction() = default;
  virtual float weight(const float& normed_errors) const;
private:
  float k_;
};

} // namespace ze
