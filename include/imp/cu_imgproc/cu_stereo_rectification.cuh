#pragma once

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/cameras/camera_models.h>

namespace ze {
namespace cu {

template<typename CameraModel,
         typename DistortionModel,
         typename Pixel>
class StereoRectifier
{
public:
  StereoRectifier(Size2u img_size,
      Eigen::Vector4f& camera_params, Eigen::Vector4f &orig_camera_params,
      Eigen::Vector4f& dist_coeffs,
      Eigen::Matrix3f& inv_H);

  ~StereoRectifier() = default;

  void rectify(ImageGpu<Pixel>& dst,
      const ImageGpu<Pixel>& src) const;

  const ImageGpu32fC2& getUndistortRectifyMap() const;

private:
  ImageGpu32fC2 undistort_rectify_map_;
  Fragmentation<16, 16> fragm_;
};

using EquidistStereoRectifier32fC1 = cu::StereoRectifier<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
using RadTanStereoRectifier32fC1 = cu::StereoRectifier<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
