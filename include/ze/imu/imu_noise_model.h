#pragma once

#include <string>

#include <ze/common/logging.hpp>
#include <ze/common/macros.h>
#include <ze/common/types.h>

namespace ze {

//----------------------------
// Noise model base class
class ImuNoiseModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuNoiseModel);

  enum NoiseTypes
  {
    None,
    WhiteBrownian
  };

  explicit ImuNoiseModel(NoiseTypes type);

  inline NoiseTypes type() const { return type_; }
  std::string typeAsString() const;

 private:
  NoiseTypes type_;
};

//----------------------------
// No noise model
class ImuNoiseNone: public ImuNoiseModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuNoiseNone);
  static constexpr ImuNoiseModel::NoiseTypes Type = ImuNoiseModel::None;

  ImuNoiseNone();
};

//----------------------------
// White brownian noise model
class ImuNoiseWhiteBrownian: public ImuNoiseModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuNoiseWhiteBrownian);
  static constexpr ImuNoiseModel::NoiseTypes Type = ImuNoiseModel::WhiteBrownian;

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
