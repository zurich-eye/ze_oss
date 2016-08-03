#include <Eigen/LU>
#include <imp/cu_core/cu_linearmemory.cuh>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_imgproc/cu_remap.cuh>
#include <imp/cu_imgproc/cu_stereo_rectification.cuh>
#include <opencv2/calib3d/calib3d.hpp>  //! @todo (MPI) get rid of OpenCV dependency

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

  // Compute map
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

  // Execute remapping
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

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
HorizontalStereoPairRectifier<CameraModel, DistortionModel, Pixel>::HorizontalStereoPairRectifier(
    Size2u img_size,
    Eigen::Vector4f& left_camera_params,
    Eigen::Vector4f& transformed_left_cam_params,
    Eigen::Vector4f& left_dist_coeffs,
    Eigen::Vector4f& right_camera_params,
    Eigen::Vector4f& transformed_right_cam_params,
    Eigen::Vector4f& right_dist_coeffs,
    Eigen::Matrix3f& R_l_r,
    Eigen::Vector3f& t_l_r,
    float& horizontal_offset)
{
  //! Currently the rectifying homography H and the
  //! transformed camera parameters are computed using
  //! cv::stereoRectify
  //! @todo (MPI) get rid of OpenCV dependency and the following ugly code
  cv::Mat_<double> cv_left_cam_params(3, 3);
  cv_left_cam_params << left_camera_params(0), 0.0, left_camera_params(2),
      0.0, left_camera_params(1), left_camera_params(3),
      0.0, 0.0, 1.0;

  cv::Mat_<double> cv_left_dist_coeffs(1, 4);
  cv_left_dist_coeffs << left_dist_coeffs(0),
      left_dist_coeffs(1), left_dist_coeffs(2), left_dist_coeffs(3);

  cv::Mat_<double> cv_right_cam_params(3, 3);
  cv_right_cam_params << right_camera_params(0), 0.0, right_camera_params(2),
      0.0, right_camera_params(1), right_camera_params(3),
      0.0, 0.0, 1.0;

  cv::Mat_<double> cv_right_dist_coeffs(1, 4);
  cv_right_dist_coeffs << right_dist_coeffs(0),
      right_dist_coeffs(1), right_dist_coeffs(2), right_dist_coeffs(3);

  cv::Mat_<double> cv_R_l_r(3, 3);
  cv_R_l_r << R_l_r(0, 0), R_l_r(0, 1), R_l_r(0, 2),
      R_l_r(1, 0), R_l_r(1, 1), R_l_r(1, 2),
      R_l_r(2, 0), R_l_r(2, 1), R_l_r(2, 2);

  cv::Mat_<double> cv_t_l_r(3, 1);
  cv_t_l_r << t_l_r(0), t_l_r(1), t_l_r(2);

  // Allocate OpenCV results
  cv::Mat cv_left_H;
  cv::Mat cv_right_H;
  cv::Mat cv_left_P;
  cv::Mat cv_right_P;
  cv::Mat disp_to_depth_Q;

  cv::stereoRectify(cv_left_cam_params, cv_left_dist_coeffs,
                    cv_right_cam_params, cv_right_dist_coeffs,
                    cv::Size(img_size.width(), img_size.height()),
                    cv_R_l_r, cv_t_l_r,
                    cv_left_H, cv_right_H,
                    cv_left_P, cv_right_P,
                    disp_to_depth_Q,
                    cv::CALIB_ZERO_DISPARITY, 0,
                    cv::Size(img_size.width(), img_size.height()));

  // Copy results from the OpenCV data structures
  Eigen::Matrix3f left_H;
  Eigen::Matrix3f right_H;

  for (uint8_t n = 0; n < 9; ++n)
  {
    const uint8_t i = n/3;
    const uint8_t j = n%3;
    left_H(i, j) = cv_left_H.at<double>(i, j);
    right_H(i, j) = cv_right_H.at<double>(i, j);
  }
  transformed_left_cam_params(0) = cv_left_P.at<double>(0, 0);
  transformed_left_cam_params(1) = cv_left_P.at<double>(1, 1);
  transformed_left_cam_params(2) = cv_left_P.at<double>(0, 2);
  transformed_left_cam_params(3) = cv_left_P.at<double>(1, 2);

  transformed_right_cam_params(0) = cv_right_P.at<double>(0, 0);
  transformed_right_cam_params(1) = cv_right_P.at<double>(1, 1);
  transformed_right_cam_params(2) = cv_right_P.at<double>(0, 2);
  transformed_right_cam_params(3) = cv_right_P.at<double>(1, 2);

  // Copy the horizontal offset (Tx) from OpeCV camera projection matrix P
  horizontal_offset = cv_right_P.at<double>(0, 3) / cv_right_P.at<double>(0, 0);

  // Allocate rectifiers for the left and right cameras
  Eigen::Matrix3f inv_left_H = left_H.inverse();
  Eigen::Matrix3f inv_right_H = right_H.inverse();
  left_rectifier_.reset(
        new StereoRectifier<CameraModel, DistortionModel, Pixel>(
          img_size, left_camera_params, transformed_left_cam_params, left_dist_coeffs, inv_left_H));
  right_rectifier_.reset(
        new StereoRectifier<CameraModel, DistortionModel, Pixel>(
          img_size, right_camera_params, transformed_right_cam_params, right_dist_coeffs, inv_right_H));
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
void HorizontalStereoPairRectifier<CameraModel, DistortionModel, Pixel>::rectify(
    ImageGpu<Pixel>& left_dst,
    ImageGpu<Pixel>& right_dst,
    const ImageGpu<Pixel>& left_src,
    const ImageGpu<Pixel>& right_src) const
{
  left_rectifier_->rectify(left_dst, left_src);
  right_rectifier_->rectify(right_dst, right_src);
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
const ImageGpu32fC2& HorizontalStereoPairRectifier<CameraModel, DistortionModel, Pixel>::getLeftCameraUndistortRectifyMap() const
{
  return left_rectifier_->getUndistortRectifyMap();
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
const ImageGpu32fC2& HorizontalStereoPairRectifier<CameraModel, DistortionModel, Pixel>::getRightCameraUndistortRectifyMap() const
{
  return right_rectifier_->getUndistortRectifyMap();
}

// Explicit template instantiations
template class HorizontalStereoPairRectifier<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
template class HorizontalStereoPairRectifier<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
