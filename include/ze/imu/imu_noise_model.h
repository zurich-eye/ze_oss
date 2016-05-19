#pragma once

#include <string>

#include <ze/common/logging.hpp>
#include <ze/common/macros.h>
#include <ze/common/types.h>

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

  ImuNoiseWhiteBrownian(FloatType noise_density,
                        FloatType bandwidth,
                        FloatType bias_noise_density);

  // getters
  inline FloatType noiseDensity() const { return noise_density_; }
  inline FloatType bandwidth() const { return bandwidth_; }
  inline FloatType biasNoiseDensity() const { return bias_noise_density_; }

private:
  FloatType noise_density_;
  FloatType bandwidth_;
  FloatType bias_noise_density_;
};

} // namespace ze
