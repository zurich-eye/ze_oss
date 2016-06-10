#pragma once
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_pinhole_camera.cuh>
#include <imp/cu_core/cu_utils.hpp>

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
      const Eigen::RowVectorXf& camera_params,
      const Eigen::RowVectorXf& dist_coeffs);
  ~ImageUndistorter() = default;
  void undistort(
      const ImageGpu<Pixel>& src,
      ImageGpu<Pixel>& dst) const;
  const ImageGpu32fC2& getUndistortionMap() const;

private:
  ImageGpu32fC2 undistortion_map_;
  Fragmentation<16, 16> fragm_;
};

using EquidistUndistort32fC1 = cu::ImageUndistorter<cu::PinholeGeometry, cu::EquidistantDistortion, Pixel32fC1>;
using RadTanUndistort32fC1 = cu::ImageUndistorter<cu::PinholeGeometry, cu::RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
