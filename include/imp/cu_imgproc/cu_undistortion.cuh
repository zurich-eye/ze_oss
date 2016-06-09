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
      const Eigen::RowVectorXf& camera_params,
      const Eigen::RowVectorXf& dist_coeffs);
  ~ImageUndistorter() = default;
  void undistort(
      const ImageGpu<Pixel>& src,
      ImageGpu<Pixel>& dst);

private:
  ImageGpu32fC2 undistortion_map_;
  Fragmentation<> fragm_;
};

} // cu namespace
} // ze namespace
