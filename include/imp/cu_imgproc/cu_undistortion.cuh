#pragma once

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/cameras/camera_models.h>

namespace ze {
namespace cu {

template<typename CameraModel,
         typename DistortionModel,
         typename Pixel>
class ImageUndistorter
{
public:
  ImageUndistorter(
      Size2u img_size,
      const VectorX& camera_params,
      const VectorX& dist_coeffs);

  ~ImageUndistorter() = default;

  void undistort(
      ImageGpu<Pixel>& dst,
      const ImageGpu<Pixel>& src) const;

  const ImageGpu32fC2& getUndistortionMap() const;

private:
  ImageGpu32fC2 undistortion_map_;
  Fragmentation<16, 16> fragm_;
};

using EquidistUndistort32fC1 = cu::ImageUndistorter<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
using RadTanUndistort32fC1 = cu::ImageUndistorter<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
