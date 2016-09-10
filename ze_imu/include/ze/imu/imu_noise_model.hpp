// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <string>

#include <ze/common/logging.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>

namespace ze {

enum class ImuNoiseType
{
  None,
  WhiteBrownian
};

//------------------------------------------------------------------------------
// Noise model base class
class ImuNoiseModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuNoiseModel);

  explicit ImuNoiseModel(ImuNoiseType type);

  inline ImuNoiseType type() const { return type_; }
  std::string typeAsString() const;

 private:
  ImuNoiseType type_;
};

//------------------------------------------------------------------------------
// No noise model
class ImuNoiseNone: public ImuNoiseModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuNoiseNone);
  static constexpr ImuNoiseType Type = ImuNoiseType::None;

  ImuNoiseNone();
};

//------------------------------------------------------------------------------
// White brownian noise model
class ImuNoiseWhiteBrownian: public ImuNoiseModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuNoiseWhiteBrownian);
  static constexpr ImuNoiseType Type = ImuNoiseType::WhiteBrownian;

  ImuNoiseWhiteBrownian(real_t noise_density,
                        real_t bandwidth,
                        real_t bias_noise_density);

  // getters
  inline real_t noiseDensity() const { return noise_density_; }
  inline real_t bandwidth() const { return bandwidth_; }
  inline real_t biasNoiseDensity() const { return bias_noise_density_; }

private:
  real_t noise_density_;
  real_t bandwidth_;
  real_t bias_noise_density_;
};

} // namespace ze
