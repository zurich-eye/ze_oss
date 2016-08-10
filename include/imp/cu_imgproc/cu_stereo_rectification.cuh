#pragma once

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/cameras/camera_models.h>
#include <ze/geometry/epipolar_geometry.hpp>

namespace ze {
namespace cu {

//! \brief A StereoRectifier can rectify images using the GPU
template<typename CameraModel,
         typename DistortionModel,
         typename Pixel>
class StereoRectifier
{
public:
  //! \brief StereoRectifier
  //! \param img_size The size of the images to rectify.
  //! \param camera_params The camera parameters [fx, fy, cx, cy]^T.
  //! \param transformed_camera_params The camera parameters
  //! [fx, fy, cx, cy]^T in the rectified reference frame.
  //! \param dist_coeffs Camera distortion coefficients.
  //! \param inv_H Inverse of the rectifying homography.
  StereoRectifier(const Size2u& img_size,
                  const Vector4& camera_params,
                  const Vector4& transformed_camera_params,
                  const Vector4& dist_coeffs,
                  const Matrix3& inv_H);

  ~StereoRectifier() = default;

  //! \brief Rectify
  //! \param dst Destination image to store rectification result in GPU memory.
  //! \param src The image to rectify in GPU memory
  void rectify(ImageGpu<Pixel>& dst,
               const ImageGpu<Pixel>& src) const;

  //! \brief Retrieves the computed undistortion-rectification map
  const ImageGpu32fC2& getUndistortRectifyMap() const;

private:
  ImageGpu32fC2 undistort_rectify_map_;   //!< The (2-channels) undistortion-rectification map
  Fragmentation<16, 16> fragm_;           //!< CUDA thread grid config
};

using EquidistStereoRectifier32fC1 = StereoRectifier<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
using RadTanStereoRectifier32fC1 = StereoRectifier<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

//! A HorizontalStereoPairRectifier is used to rectify images acquired by a fully calibrated
//! camera pair in horizontal stereo setting. The left camera is used as reference.
template<typename CameraModel,
         typename DistortionModel,
         typename Pixel>
class HorizontalStereoPairRectifier
{
public:
  //! \brief HorizontalStereoPairRectifier
  //! \param transformed_left_cam_params The output left camera parameters [fx, fy, cx, cy]^T
  //! \param transformed_right_cam_params The output right camera parameters [fx, fy, cx, cy]^T
  //! \param horizontal_offset Output horizontal offset in the rectified reference system.
  //! \param img_size The size of the images to rectify.
  //! \param left_camera_params The camera parameters [fx, fy, cx, cy]^T for the left camera
  //! for the left camera in the rectified reference frame.
  //! \param left_dist_coeffs The distortion coefficients for the left camera
  //! \param right_camera_params The camera parameters [fx, fy, cx, cy]^T for the right camera
  //! for the right camera in the rectified reference frame.
  //! \param right_dist_coeffs The distortion coefficients for the right camera.
  //! \param T_l_r transformation from "Right" to "Left" reference system.
  HorizontalStereoPairRectifier(Vector4& transformed_left_cam_params,
                                Vector4& transformed_right_cam_params,
                                FloatType& horizontal_offset,
                                const Size2u& img_size,
                                const Vector4& left_camera_params,
                                const Vector4& left_dist_coeffs,
                                const Vector4& right_camera_params,
                                const Vector4& right_dist_coeffs,
                                const Transformation& T_L_R);

  ~HorizontalStereoPairRectifier() = default;

  //! \brief Run rectification
  //! \param left_dst Destination image to store the rectified left camera image in GPU memory.
  //! \param right_dst Destination image to store the rectified right camera image in GPU memory.
  //! \param left_src The left source image in GPU memory
  //! \param right_src The right source image in GPU memory
  void rectify(ImageGpu<Pixel>& left_dst,
               ImageGpu<Pixel>& right_dst,
               const ImageGpu<Pixel>& left_src,
               const ImageGpu<Pixel>& right_src) const;

  //! \brief Retrieves the computed undistortion-rectification maps
  const ImageGpu32fC2& getLeftCameraUndistortRectifyMap() const;
  const ImageGpu32fC2& getRightCameraUndistortRectifyMap() const;

private:
  std::unique_ptr<StereoRectifier<CameraModel, DistortionModel, Pixel>> left_rectifier_;
  std::unique_ptr<StereoRectifier<CameraModel, DistortionModel, Pixel>> right_rectifier_;
};

using HorizontalStereoPairRectifierEquidist32fC1 = HorizontalStereoPairRectifier<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
using HorizontalStereoPairRectifierRadTan32fC1 = HorizontalStereoPairRectifier<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
