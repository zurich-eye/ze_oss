#include <imp/cu_core/cu_linearmemory.cuh>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_imgproc/cu_remap.cuh>
#include <imp/cu_imgproc/cu_stereo_rectification.cuh>

namespace ze {
namespace cu {

template<typename CameraModel,
         typename DistortionModel>
__global__
void k_computeUndistortRectifyMap(
    Pixel32fC2* dst,
    size_t dst_stride,
    std::uint32_t width,
    std::uint32_t height,
    const float* d_cam_params,
    const float* d_orig_cam_params,
    const float* d_dist_coeffs,
    const float* d_inv_H)
{
  const int u = blockIdx.x*blockDim.x + threadIdx.x;
  const int v = blockIdx.y*blockDim.y + threadIdx.y;

  if (u < width && v < height)
  {
    float px[2]{static_cast<float>(u), static_cast<float>(v)};
    CameraModel::backProject(d_cam_params, px);
    const float x = d_inv_H[0]*px[0]+d_inv_H[3]*px[1]+d_inv_H[6];
    const float y = d_inv_H[1]*px[0]+d_inv_H[4]*px[1]+d_inv_H[7];
    const float w = d_inv_H[2]*px[0]+d_inv_H[5]*px[1]+d_inv_H[8];
    px[0] = x / w;
    px[1] = y / w;
    DistortionModel::distort(d_dist_coeffs, px);
    CameraModel::project(d_orig_cam_params, px);
    dst[v*dst_stride + u][0] = px[0];
    dst[v*dst_stride + u][1] = px[1];
  }
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
StereoRectifier<CameraModel, DistortionModel, Pixel>::StereoRectifier(
    Size2u img_size,
    Eigen::Vector4f& camera_params,
    Eigen::Vector4f& transformed_camera_params,
    Eigen::Vector4f& dist_coeffs,
    Eigen::Matrix3f& inv_H)
  : undistort_rectify_map_(img_size)
  , fragm_(img_size)
{
  // Upload to GPU
  ze::LinearMemory32fC1 h_cam_params(
        reinterpret_cast<Pixel32fC1*>(camera_params.data()),
        4, true);
  ze::LinearMemory32fC1 h_transformed_cam_params(
        reinterpret_cast<Pixel32fC1*>(transformed_camera_params.data()),
        4, true);
  ze::LinearMemory32fC1 h_dist_coeffs(
        reinterpret_cast<Pixel32fC1*>(dist_coeffs.data()),
        4, true);
  ze::LinearMemory32fC1 h_inv_H(
        reinterpret_cast<Pixel32fC1*>(inv_H.data()),
        9, true);
  cu::LinearMemory32fC1 d_cam_params(h_cam_params);
  cu::LinearMemory32fC1 d_transformed_cam_params(h_transformed_cam_params);
  cu::LinearMemory32fC1 d_dist_coeffs(h_dist_coeffs);
  cu::LinearMemory32fC1 d_inv_H(h_inv_H);

  k_computeUndistortRectifyMap<CameraModel, DistortionModel>
      <<<
        fragm_.dimGrid, fragm_.dimBlock
      >>> (undistort_rectify_map_.data(),
           undistort_rectify_map_.stride(),
           undistort_rectify_map_.width(),
           undistort_rectify_map_.height(),
           d_cam_params.cuData(),
           d_transformed_cam_params.cuData(),
           d_dist_coeffs.cuData(),
           d_inv_H.cuData());
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
void StereoRectifier<CameraModel, DistortionModel, Pixel>::rectify(
    ImageGpu<Pixel>& dst,
    const ImageGpu<Pixel>& src) const
{
  CHECK_EQ(src.size(), dst.size());
  CHECK_EQ(src.size(), undistort_rectify_map_.size());

  std::shared_ptr<Texture2D> src_tex =
      src.genTexture(false, cudaFilterModeLinear);
  IMP_CUDA_CHECK();

  k_remap
      <<<
        fragm_.dimGrid, fragm_.dimBlock
      >>> (dst.data(),
           dst.stride(),
           undistort_rectify_map_.data(),
           undistort_rectify_map_.stride(),
           dst.width(),
           dst.height(),
           *src_tex);
  IMP_CUDA_CHECK();
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
const ImageGpu32fC2& StereoRectifier<CameraModel, DistortionModel, Pixel>::getUndistortRectifyMap() const
{
  return undistort_rectify_map_;
}

// Explicit template instantiations
template class StereoRectifier<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
template class StereoRectifier<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
