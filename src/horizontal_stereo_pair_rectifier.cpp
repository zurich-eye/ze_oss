#include <Eigen/LU>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_imgproc/horizontal_stereo_pair_rectifier.hpp>
#include <opencv2/calib3d/calib3d.hpp>  //! @todo (MPI) get rid of OpenCV dependency

namespace ze {
namespace cu {

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
HorizontalStereoPairRectifier<CameraModel, DistortionModel, Pixel>::HorizontalStereoPairRectifier(
    Size2u img_size,
    Eigen::Vector4f& left_camera_params,
    Eigen::Vector4f& left_dist_coeffs,
    Eigen::Vector4f& right_camera_params,
    Eigen::Vector4f& right_dist_coeffs,
    Eigen::Matrix3f& R_l_r,
    Eigen::Vector3f& t_l_r)
{
  //! Currently the rectifying homography H and the
  //! transformed camera parameters are computed using
  //! cv::stereoRectify
  //! @todo (MPI) get rid of OpenCV dependency
  cv::Mat_<double> cv_left_cam_params(3, 3);
  cv_left_cam_params << left_camera_params(0),
      left_camera_params(1), left_camera_params(2), left_camera_params(3);

  cv::Mat_<double> cv_left_dist_coeffs(1, 4);
  cv_left_dist_coeffs << left_dist_coeffs(0),
      left_dist_coeffs(1), left_dist_coeffs(2), left_dist_coeffs(3);

  cv::Mat_<double> cv_right_cam_params(3, 3);
  cv_right_cam_params << right_camera_params(0),
      right_camera_params(1), right_camera_params(2), right_camera_params(3);

  cv::Mat_<double> cv_right_dist_coeffs(1, 4);
  cv_right_dist_coeffs << right_dist_coeffs(0),
      right_dist_coeffs(1), right_dist_coeffs(2), right_dist_coeffs(3);

  cv::Mat_<double> cv_R_l_r;
  cv_R_l_r << R_l_r(0, 0), R_l_r(0, 1), R_l_r(0, 2),
      R_l_r(1, 0), R_l_r(1, 1), R_l_r(1, 2),
      R_l_r(2, 0), R_l_r(2, 1), R_l_r(2, 2);

  cv::Mat_<double> cv_t_l_r;
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

  Eigen::Matrix3f left_H;
  Eigen::Matrix3f right_H;
  Eigen::Vector4f transformed_left_cam_params;
  Eigen::Vector4f transformed_right_cam_params;
  for (uint8_t n = 0; n < 9; ++n)
  {
    uint8_t i = n/3;
    uint8_t j = n%3;
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

// Explicit template instantiations
template class HorizontalStereoPairRectifier<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
template class HorizontalStereoPairRectifier<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
