#include <imp/cu_imgproc/cu_undistortion.cuh>

#include <imp/cu_core/cu_texture.cuh>

namespace ze {
namespace cu {

template <typename CameraModel,
          typename DistortionModel,
          typename T>
__global__
void k_computeUndistortionMap(
    Pixel32fC2* dst,
    size_t dst_stride,
    std::uint32_t width,
    std::uint32_t height,
    T* d_cam_params,
    T* d_dist_coeffs)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x < width && y < height)
  {
    float px[2];
    px[0] = x;
    px[1] = y;
    // Pixel32fC2 const * px = dst + y*dst_stride + x;
    CameraModel::backProject(d_cam_params, px);
    DistortionModel::distort(d_dist_coeffs, px);
    CameraModel::project(d_cam_params, px);
    dst[y*dst_stride + x][0] = px[0];
    dst[y*dst_stride + x][1] = px[1];
  }
}

__global__
void k_undistort(
    Pixel32fC1* dst,
    size_t dst_stride,
    Pixel32fC2* map,
    size_t map_stride,
    std::uint32_t width,
    std::uint32_t height,
    Texture2D src)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x < width && y < height)
  {
    Pixel32fC1 val;
    const Pixel32fC2& px = map[y*map_stride+x];
    tex2DFetch(val, src, px[0], px[1]);
    dst[y*dst_stride+x] = val;
  }
}

template <typename CameraModel,
          typename DistortionModel,
          typename T>
ImageUndistorter<CameraModel, DistortionModel, T>::ImageUndistorter(
    int32_t width, int32_t height,
    T* camera_params, T* dist_coeffs)
  : undistortion_map_(width, height),
    fragm_(width, height)
{
  cudaMalloc(&d_cam_params_, 4*sizeof(T));
  cudaMalloc(&d_dist_coeffs_, 4*sizeof(T));
  cudaMemcpy(d_cam_params_, camera_params, 4*sizeof(T), cudaMemcpyHostToDevice);
  cudaMemcpy(d_dist_coeffs_, dist_coeffs, 4*sizeof(T), cudaMemcpyHostToDevice);

  k_computeUndistortionMap<CameraModel, DistortionModel, T>
      <<<
        fragm_.dimGrid, fragm_.dimBlock
      >>> (undistortion_map_.data(),
           undistortion_map_.stride(),
           undistortion_map_.width(),
           undistortion_map_.height(),
           d_cam_params_,
           d_dist_coeffs_);
}

template <typename CameraModel,
          typename DistortionModel,
          typename T>
ImageUndistorter<CameraModel, DistortionModel, T>::~ImageUndistorter()
{
}

template <typename CameraModel,
          typename DistortionModel,
          typename T>
void ImageUndistorter<CameraModel, DistortionModel, T>::undistort(
    const ImageGpu32fC1& in,
    ImageGpu32fC1& out)
{
  // Add checks if map and out images have the same size


  std::shared_ptr<Texture2D> src_tex = in.genTexture(false, cudaFilterModeLinear);
  k_undistort
      <<<
        fragm_.dimGrid, fragm_.dimBlock
      >>> (out.data(),
           out.stride(),
           undistortion_map_.data(),
           undistortion_map_.stride(),
           out.width(),
           out.height(),
           *src_tex);
}

template class ImageUndistorter<PinholeGeometry, EquidistantDistortion, float>;

} // cu namespace
} // ze namespace
