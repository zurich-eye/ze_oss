#pragma once
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/cameras/camera_models.h>

namespace ze {
namespace cu {

template <typename CameraModel,
          typename DistortionModel,
          typename T>
class ImageUndistorter
{
public:
  ImageUndistorter(int32_t width, int32_t height, T* camera_params, T* dist_coeffs);
  ~ImageUndistorter();
  void undistort(const ImageGpu32fC1& in, ImageGpu32fC1& out);

private:
  ImageGpu32fC2 undistortion_map_;
  T* d_cam_params_;
  T* d_dist_coeffs_;

  Fragmentation<> fragm_;
};

} // cu namespace
} // ze namespace
