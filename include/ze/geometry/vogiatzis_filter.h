// FROM SVO

#pragma once

#include <ze/common/statistics.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>

#include <glog/logging.h>

namespace ze {

using SeedState = Vector4;

inline FloatType getDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return 1.0 / mu_sigma2_a_b(0);
}

inline FloatType getInvDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(0);
}

inline FloatType getInvMinDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(0) + std::sqrt(mu_sigma2_a_b(1));
}

inline FloatType getInvMaxDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return std::max(mu_sigma2_a_b(0) - std::sqrt(mu_sigma2_a_b(1)), FloatType{0.00000001});
}

inline FloatType getMeanFromDepth(FloatType depth)
{
  return 1.0 / depth;
}

inline FloatType getMeanRangeFromDepthMinMax(FloatType depth_min, FloatType /*depth_max*/)
{
  return 1.0 / depth_min;
}

inline FloatType getInitSigma2FromMuRange(FloatType mu_range)
{
  return mu_range * mu_range / 36.0;
}

inline void increaseOutlierProbability(Eigen::Ref<SeedState> mu_sigma2_a_b)
{
  mu_sigma2_a_b(3) += 1;
}

inline bool isConverged(
    const Eigen::Ref<const SeedState>& mu_sigma2_a_b,
    FloatType mu_range, FloatType sigma2_convergence_threshold)
{
  // If initial uncertainty was reduced by factor sigma2_convergence_threshold
  // we accept the seed as converged.
  const FloatType thresh = mu_range / sigma2_convergence_threshold;
  return (mu_sigma2_a_b(1) < thresh * thresh);
}

inline FloatType getSigma2FromDepthSigma(FloatType depth, FloatType depth_sigma)
{
  const FloatType sigma = 0.5 * (1.0 / std::max(0.000000000001, depth - depth_sigma)
                               - 1.0 / (depth + depth_sigma));
  return sigma * sigma;
}

inline bool updateFilterVogiatzis(
    const FloatType z, // Measurement
    const FloatType tau2,
    const FloatType mu_range,
    Eigen::Ref<SeedState> mu_sigma2_a_b)
{
  FloatType& mu = mu_sigma2_a_b(0);
  FloatType& sigma2 = mu_sigma2_a_b(1);
  FloatType& a = mu_sigma2_a_b(2);
  FloatType& b = mu_sigma2_a_b(3);

  const FloatType norm_scale = std::sqrt(sigma2 + tau2);
  if(std::isnan(norm_scale))
  {
    LOG(WARNING) << "Update Seed: Sigma2+Tau2 is NaN";
    return false;
  }

  const FloatType oldsigma2 = sigma2;
  const FloatType s2 = 1.0/(1.0/sigma2 + 1.0/tau2);
  const FloatType m = s2*(mu/sigma2 + z/tau2);
  const FloatType uniform_x = 1.0/mu_range;
  FloatType C1 = a / (a + b) * normPdf<FloatType>(z, mu, norm_scale);
  FloatType C2 = b / (a + b) * uniform_x;
  const FloatType normalization_constant = C1 + C2;
  C1 /= normalization_constant;
  C2 /= normalization_constant;
  const FloatType f = C1 * (a+1.0) / (a+b+1.0) + C2 * a/(a+b+1.0);
  const FloatType e = C1 * (a+1.0)*(a+2.0) / ((a+b+1.0) * (a+b+2.0))
                    + C2 * a * (a+1.0) / ((a+b+1.0)*(a+b+2.0));

  // update parameters
  const FloatType mu_new = C1*m+C2*mu;
  sigma2 = C1*(s2 + m*m) + C2*(sigma2 + mu*mu) - mu_new*mu_new;
  mu = mu_new;
  a = (e - f) / (f - e / f);
  b = a * (1.0 - f) / f;

  // TODO: This happens sometimes.
  if(sigma2 < 0.0)
  {
    LOG(WARNING) << "Seed sigma2 is negative!";
    sigma2 = oldsigma2;
  }
  if(mu < 0.0)
  {
    LOG(WARNING) << "Seed diverged! mu is negative!!";
    mu = 1.0;
    return false;
  }
  if(std::isnan(mu))
  {
    LOG(WARNING) << "Seed is NaN";
    return false;
  }
  return true;
}

inline bool updateFilterGaussian(
    const FloatType z, // Measurement
    const FloatType tau2,
    Eigen::Ref<SeedState> mu_sigma2_a_b)
{
  FloatType& mu = mu_sigma2_a_b(0);
  FloatType& sigma2 = mu_sigma2_a_b(1);
  FloatType& a = mu_sigma2_a_b(2);
  FloatType& b = mu_sigma2_a_b(3);

  const FloatType norm_scale = std::sqrt(sigma2 + tau2);
  if(std::isnan(norm_scale))
  {
    LOG(WARNING) << "Update Seed: Sigma2+Tau2 is NaN";
    return false;
  }

  const FloatType denom = (sigma2 + tau2);
  mu = (sigma2 * z + tau2 * mu) / denom;
  sigma2 = sigma2 * tau2 / denom;

  CHECK_GE(sigma2, 0.0);
  CHECK_GE(mu, 0.0);
  return true;
}

inline FloatType computeTau(
      const Transformation& T_ref_cur,
      const Bearing& f,
      const FloatType z,
      const FloatType px_error_angle)
{
  const Bearing& t = T_ref_cur.getPosition();
  const Bearing a = f * z - t;
  FloatType t_norm = t.norm();
  FloatType a_norm = a.norm();
  FloatType alpha = std::acos(f.dot(t) / t_norm); // dot product
  FloatType beta = std::acos(a.dot(-t) / (t_norm * a_norm)); // dot product
  FloatType beta_plus = beta + px_error_angle;
  FloatType gamma_plus = M_PI - alpha - beta_plus; // triangle angles sum to PI
  FloatType z_plus = t_norm * std::sin(beta_plus) / std::sin(gamma_plus); // law of sines
  return (z_plus - z); // tau
}

} // namespace ze
