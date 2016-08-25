// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

// from SVO

#pragma once

#include <ze/common/statistics.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

#include <glog/logging.h>

namespace ze {

inline real_t getDepth(const Eigen::Ref<const Seed>& mu_sigma2_a_b)
{
  return real_t{1.0} / mu_sigma2_a_b(0);
}

inline real_t getSigma2(const Eigen::Ref<const Seed>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(1);
}

inline real_t getInvDepth(const Eigen::Ref<const Seed>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(0);
}

inline real_t getInvMinDepth(const Eigen::Ref<const Seed>& mu_sigma2_a_b)
{
  return mu_sigma2_a_b(0) + std::sqrt(mu_sigma2_a_b(1));
}

inline real_t getInvMaxDepth(const Eigen::Ref<const Seed>& mu_sigma2_a_b)
{
  return std::max(mu_sigma2_a_b(0) - std::sqrt(mu_sigma2_a_b(1)), real_t{0.00000001});
}

inline real_t getMeanFromDepth(real_t depth)
{
  return real_t{1.0} / depth;
}

inline real_t getMeanRangeFromDepthMinMax(real_t depth_min, real_t /*depth_max*/)
{
  return real_t{1.0} / depth_min;
}

inline real_t getInitSigma2FromMuRange(real_t mu_range)
{
  return mu_range * mu_range / real_t{36.0};
}

inline void increaseOutlierProbability(Eigen::Ref<Seed> mu_sigma2_a_b)
{
  mu_sigma2_a_b(3) += real_t{1.0};
}

inline bool isConverged(
    const Eigen::Ref<const Seed>& mu_sigma2_a_b,
    real_t mu_range, real_t sigma2_convergence_threshold)
{
  // If initial uncertainty was reduced by factor sigma2_convergence_threshold
  // we accept the seed as converged.
  const real_t thresh = mu_range / sigma2_convergence_threshold;
  return (mu_sigma2_a_b(1) < thresh * thresh);
}

inline real_t getSigma2FromDepthSigma(real_t depth, real_t depth_sigma)
{
  const real_t sigma =
      real_t{0.5} * (real_t{1.0} / std::max(real_t{1.0e-8}, depth - depth_sigma)
                      - real_t{1.0} / (depth + depth_sigma));
  return sigma * sigma;
}

inline bool updateFilterVogiatzis(
    const real_t z, // Measurement
    const real_t tau2,
    const real_t mu_range,
    Eigen::Ref<Seed> mu_sigma2_a_b)
{
  real_t& mu = mu_sigma2_a_b(0);
  real_t& sigma2 = mu_sigma2_a_b(1);
  real_t& a = mu_sigma2_a_b(2);
  real_t& b = mu_sigma2_a_b(3);

  const real_t norm_scale = std::sqrt(sigma2 + tau2);
  if(std::isnan(norm_scale))
  {
    LOG(WARNING) << "Update Seed: Sigma2+Tau2 is NaN";
    return false;
  }

  const real_t oldsigma2 = sigma2;
  const real_t s2 = real_t{1.0} / (real_t{1.0} / sigma2 + real_t{1.0} / tau2);
  const real_t m = s2 * (mu / sigma2 + z / tau2);
  const real_t uniform_x = real_t{1.0} / mu_range;
  real_t C1 = a / (a + b) * normPdf<real_t>(z, mu, norm_scale);
  real_t C2 = b / (a + b) * uniform_x;
  const real_t normalization_constant = C1 + C2;
  C1 /= normalization_constant;
  C2 /= normalization_constant;
  const real_t f =
      C1 * (a+real_t{1.0}) / (a+b+real_t{1.0}) + C2 * a/(a+b+real_t{1.0});
  const real_t e =
      C1 * (a+real_t{1.0}) * (a+real_t{2.0}) / ((a+b+real_t{1.0}) * (a+b+real_t{2.0}))
    + C2 * a * (a+real_t{1.0}) / ((a+b+real_t{1.0})*(a+b+real_t{2.0}));

  // update parameters
  const real_t mu_new = C1 * m + C2 * mu;
  sigma2 = C1 * (s2 + m * m) + C2 * (sigma2 + mu * mu) - mu_new * mu_new;
  mu = mu_new;
  a = (e - f) / (f - e / f);
  b = a * (real_t{1.0} - f) / f;

  //! @todo Check if this happens sometimes.
  if(sigma2 < real_t{0.0})
  {
    LOG(WARNING) << "Seed sigma2 is negative!";
    sigma2 = oldsigma2;
  }
  if(mu < real_t{0.0})
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
    const real_t z, // Measurement
    const real_t tau2,
    Eigen::Ref<Seed> mu_sigma2_a_b)
{
  real_t& mu = mu_sigma2_a_b(0);
  real_t& sigma2 = mu_sigma2_a_b(1);

  const real_t norm_scale = std::sqrt(sigma2 + tau2);
  if(std::isnan(norm_scale))
  {
    LOG(WARNING) << "Update Seed: Sigma2+Tau2 is NaN";
    return false;
  }

  const real_t denom = (sigma2 + tau2);
  mu = (sigma2 * z + tau2 * mu) / denom;
  sigma2 = sigma2 * tau2 / denom;

  CHECK_GE(sigma2, 0.0);
  CHECK_GE(mu, 0.0);
  return true;
}

inline real_t computeTau(
      const Transformation& T_ref_cur,
      const Bearing& f,
      const real_t z,
      const real_t px_error_angle)
{
  const Bearing& t = T_ref_cur.getPosition();
  const Bearing a = f * z - t;
  real_t t_norm = t.norm();
  real_t a_norm = a.norm();
  real_t alpha = std::acos(f.dot(t) / t_norm); // dot product
  real_t beta = std::acos(a.dot(-t) / (t_norm * a_norm)); // dot product
  real_t beta_plus = beta + px_error_angle;
  real_t gamma_plus = M_PI - alpha - beta_plus; // triangle angles sum to PI
  real_t z_plus = t_norm * std::sin(beta_plus) / std::sin(gamma_plus); // law of sines
  return (z_plus - z); // tau
}

} // namespace ze
