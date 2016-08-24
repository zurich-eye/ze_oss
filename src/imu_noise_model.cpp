// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include "ze/imu/imu_noise_model.h"

namespace ze {

//------------------------------------------------------------------------------
// Noise model base class

ImuNoiseModel::ImuNoiseModel(ImuNoiseType type)
  : type_(type)
{
}

std::string ImuNoiseModel::typeAsString() const
{
  switch (type())
  {
    case ImuNoiseType::WhiteBrownian: return "White Brownian";
    case ImuNoiseType::None: return "No Noise";
    default:
      LOG(FATAL) << "Unknown noise model";
  }
}

//------------------------------------------------------------------------------
// No Noise model
ImuNoiseNone::ImuNoiseNone(): ImuNoiseModel(Type)
{
}

//------------------------------------------------------------------------------
// White brownian noise model
ImuNoiseWhiteBrownian::ImuNoiseWhiteBrownian(real_t noise_density,
           real_t bandwidth,
           real_t bias_noise_density)
  : ImuNoiseModel(Type)
  , noise_density_(noise_density)
  , bandwidth_(bandwidth)
  , bias_noise_density_(bias_noise_density)
{
  CHECK(bandwidth > 0) << "Bandwidth must be >0'";
}

} // namespace ze
