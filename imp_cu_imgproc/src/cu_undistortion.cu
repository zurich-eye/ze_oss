// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include <imp/cu_core/cu_linearmemory.cuh>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_imgproc/cu_remap.cuh>
#include <imp/cu_imgproc/cu_undistortion.cuh>

namespace ze {
namespace cu {

template<typename CameraModel,
         typename DistortionModel>
__global__
void k_computeUndistortionMap(
    Pixel32fC2* dst,
    size_t dst_stride,
    uint32_t width,
    uint32_t height,
    const float* d_cam_params,
    const float* d_dist_coeffs)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x < width && y < height)
  {
    float px[2]{static_cast<float>(x), static_cast<float>(y)};
    CameraModel::backProject(d_cam_params, px);
    DistortionModel::distort(d_dist_coeffs, px);
    CameraModel::project(d_cam_params, px);
    dst[y*dst_stride + x][0] = px[0];
    dst[y*dst_stride + x][1] = px[1];
  }
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
ImageUndistorter<CameraModel, DistortionModel, Pixel>::ImageUndistorter(
    const Size2u& img_size,
    const VectorX& camera_params,
    const VectorX& dist_coeffs)
  : undistortion_map_(img_size),
    fragm_(img_size)
{
  // Cast to float and allocate host linear memory
  Eigen::VectorXf cp_flt = camera_params.cast<float>();
  Eigen::VectorXf dist_flt = dist_coeffs.cast<float>();
  ze::LinearMemory32fC1 h_cam_params(
        reinterpret_cast<Pixel32fC1*>(cp_flt.data()),
        camera_params.rows(), true);
  ze::LinearMemory32fC1 h_dist_coeffs(
        reinterpret_cast<Pixel32fC1*>(dist_flt.data()),
        dist_coeffs.rows(), true);
  // Copy to device linear memory
  cu::LinearMemory32fC1 d_cam_params(h_cam_params);
  cu::LinearMemory32fC1 d_dist_coeffs(h_dist_coeffs);

  k_computeUndistortionMap<CameraModel, DistortionModel>
      <<<
        fragm_.dimGrid, fragm_.dimBlock
      >>> (undistortion_map_.data(),
           undistortion_map_.stride(),
           undistortion_map_.width(),
           undistortion_map_.height(),
           d_cam_params.cuData(),
           d_dist_coeffs.cuData());
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
void ImageUndistorter<CameraModel, DistortionModel, Pixel>::undistort(
    ImageGpu<Pixel>& dst,
    const ImageGpu<Pixel>& src) const
{
  CHECK_EQ(src.size(), dst.size());
  CHECK_EQ(src.size(), undistortion_map_.size());
  std::shared_ptr<Texture2D> src_tex =
      src.genTexture(false, cudaFilterModeLinear);
  IMP_CUDA_CHECK();
  k_remap
      <<<
        fragm_.dimGrid, fragm_.dimBlock
      >>> (dst.data(),
           dst.stride(),
           undistortion_map_.data(),
           undistortion_map_.stride(),
           dst.width(),
           dst.height(),
           *src_tex);
  IMP_CUDA_CHECK();
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
const ImageGpu32fC2& ImageUndistorter<CameraModel, DistortionModel, Pixel>::getUndistortionMap() const
{
  return undistortion_map_;
}

// Explicit template instantiations
template class ImageUndistorter<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
template class ImageUndistorter<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
