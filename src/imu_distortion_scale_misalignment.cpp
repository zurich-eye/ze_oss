#include <ze/imu/imu_distortion_scale_misalignment.h>

namespace ze {

ImuDistortionScaleMisalignment::ImuDistortionScaleMisalignment()
: ImuDistortion(ImuDistortion::Type::kScaleMisalignment)
{
}

void ImuDistortionScaleMisalignment::undistort(ImuAccGyr& measurements)
{
}

} // namespace ze
