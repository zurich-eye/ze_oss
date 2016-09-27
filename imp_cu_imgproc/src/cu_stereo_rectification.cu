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
#include <imp/cu_imgproc/cu_stereo_rectification.cuh>
#include <ze/geometry/epipolar_geometry.hpp>

namespace ze {
namespace cu {

//! @todo (MPI) test constant memory fo camera/dist parameters
//! maybe also for the rectifying homography
template<typename CameraModel,
         typename DistortionModel>
__global__
void k_computeUndistortRectifyMap(
    Pixel32fC2* dst,
    size_t dst_stride,
    std::uint32_t width,
    std::uint32_t height,
    const float* d_cam_params,
    const float* d_transformed_cam_params,
    const float* d_dist_coeffs,
    const float* d_inv_H)
{
  const int u = blockIdx.x*blockDim.x + threadIdx.x;
  const int v = blockIdx.y*blockDim.y + threadIdx.y;

  if (u < width && v < height)
  {
    float px[2]{static_cast<float>(u), static_cast<float>(v)};
    CameraModel::backProject(d_transformed_cam_params, px);
    const float x = d_inv_H[0]*px[0]+d_inv_H[3]*px[1]+d_inv_H[6];
    const float y = d_inv_H[1]*px[0]+d_inv_H[4]*px[1]+d_inv_H[7];
    const float w = d_inv_H[2]*px[0]+d_inv_H[5]*px[1]+d_inv_H[8];
    px[0] = x / w;
    px[1] = y / w;
    DistortionModel::distort(d_dist_coeffs, px);
    CameraModel::project(d_cam_params, px);
    dst[v*dst_stride + u][0] = px[0];
    dst[v*dst_stride + u][1] = px[1];
  }
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
StereoRectifier<CameraModel, DistortionModel, Pixel>::StereoRectifier(
    const Size2u& img_size,
    const Vector4& camera_params,
    const Vector4& transformed_camera_params,
    const Vector4& dist_coeffs,
    const Matrix3& inv_H)
  : undistort_rectify_map_(img_size)
  , fragm_(img_size)
{
  //! Convert to float and upload to GPU
  Eigen::Vector4f cp_flt = camera_params.cast<float>();
  Eigen::Vector4f tcp_flt = transformed_camera_params.cast<float>();
  Eigen::Vector4f dist_flt = dist_coeffs.cast<float>();
  Eigen::Matrix3f inv_H_flt = inv_H.cast<float>();
  //! Copy to host LinearMemory
  ze::LinearMemory32fC1 h_cam_params(
        reinterpret_cast<Pixel32fC1*>(cp_flt.data()),
        4, true);
  ze::LinearMemory32fC1 h_transformed_cam_params(
        reinterpret_cast<Pixel32fC1*>(tcp_flt.data()),
        4, true);
  ze::LinearMemory32fC1 h_dist_coeffs(
        reinterpret_cast<Pixel32fC1*>(dist_flt.data()),
        4, true);
  ze::LinearMemory32fC1 h_inv_H(
        reinterpret_cast<Pixel32fC1*>(inv_H_flt.data()),
        9, true);
  //! Copy to device LinearMemory
  cu::LinearMemory32fC1 d_cam_params(h_cam_params);
  cu::LinearMemory32fC1 d_transformed_cam_params(h_transformed_cam_params);
  cu::LinearMemory32fC1 d_dist_coeffs(h_dist_coeffs);
  cu::LinearMemory32fC1 d_inv_H(h_inv_H);

  //! Compute map
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

  // Attach texture
  std::shared_ptr<Texture2D> src_tex =
      src.genTexture(false, cudaFilterModeLinear);
  IMP_CUDA_CHECK();

  //! Execute remapping
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
