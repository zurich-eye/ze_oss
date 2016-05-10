#include <ze/imu/imu_factory.h>

#include <ze/imu/imu.h>
#include <ze/imu/imu_distortion.h>
#include <ze/imu/imu_distortion_scale_misalignment.h>

namespace ze {

Imu::Ptr createImu(ImuDistortion::Type distortion_type) {

  ImuDistortion::UniquePtr distortion;

  switch(distortion_type) {
    case ImuDistortion::Type::kIdeal:
      break;
    case ImuDistortion::Type::kScaleMisalignment:
      break;
    case ImuDistortion::Type::kScaleMisalignmentSizeEffect:
      break;
    default:
      LOG(FATAL) << "Unknown intrinsic imu model: "
      << static_cast<std::underlying_type<ImuDistortion::Type>::type>(
          distortion_type);
  }

  Imu::Ptr imu;

  return imu;
}

}  // namespace ze
