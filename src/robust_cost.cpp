#include <ze/geometry/robust_cost.h>

#include <cmath>
#include <numeric>
#include <algorithm>
#include <glog/logging.h>

#include <ze/common/stl_utils.h>

namespace ze {

// -----------------------------------------------------------------------------
// Scale Estimators
float UnitScaleEstimator::compute(std::vector<float>& /*errors*/) const
{
  return 1.0f;
}

float MADScaleEstimator::compute(std::vector<float>& errors) const
{

}

float NormalDistributionScaleEstimator::compute(std::vector<float>& errors) const
{

}

//------------------------------------------------------------------------------
// Weight Functions
float UnitWeightFunction::weight(const float& error) const
{
  return 1.0f;
}

TukeyWeightFunction::TukeyWeightFunction(const float b)
  : b_square_(b*b)
{}

float TukeyWeightFunction::weight(const float& error) const
{
  const float x_square = error * error;
  if(x_square <= b_square_)
  {
    const float tmp = 1.0f - x_square / b_square_;
    return tmp * tmp;
  }
  else
  {
    return 0.0f;
  }
}

HuberWeightFunction::HuberWeightFunction(const float k)
  : k_(k)
{}

float HuberWeightFunction::weight(const float& error) const
{
  const float abs_error = std::fabs(error);
  return (abs_error < k_) ? 1.0f : k_/abs_error;
}

} // namespace ze


