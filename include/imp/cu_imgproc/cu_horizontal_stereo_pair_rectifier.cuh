#pragma once

#include <imp/cu_imgproc/cu_stereo_rectification.cuh>
#include <ze/geometry/epipolar_geometry.hpp>

namespace ze {
namespace cu {

//! A HorizontalStereoPairRectifier is used to rectify images acquired by a fully calibrated
//! camera pair in horizontal stereo setting.
template<typename CameraModel,
         typename DistortionModel,
         typename Pixel>
class HorizontalStereoPairRectifier
{
public:
  //! \brief HorizontalStereoPairRectifier
  //! \param transformed_cam0_params The output camera 0 parameters [fx, fy, cx, cy]^T
  //! in the rectified reference frame.
  //! \param transformed_cam1_params The output camera 1 parameters [fx, fy, cx, cy]^T
  //! in the rectified reference frame.
  //! \param horizontal_offset Output horizontal offset in the rectified reference system.
  //! \param img_size The size of the images to rectify.
  //! \param cam0_params The camera parameters [fx, fy, cx, cy]^T for the camera 0
  //! \param cam0_dist_coeffs The distortion coefficients for the camera 0
  //! \param cam1_params The camera parameters [fx, fy, cx, cy]^T for the camera 1
  //! \param cam1_dist_coeffs The distortion coefficients for the camera 1.
  //! \param T_cam1_cam0 transformation from cam0 to cam1 reference system.
  HorizontalStereoPairRectifier(Vector4& transformed_cam0_params,
                                Vector4& transformed_cam1_params,
                                real_t& horizontal_offset,
                                const Size2u& img_size,
                                const Vector4& cam0_params,
                                const Vector4& cam0_dist_coeffs,
                                const Vector4& cam1_params,
                                const Vector4& cam1_dist_coeffs,
                                const Transformation& T_cam1_cam0);

  ~HorizontalStereoPairRectifier() = default;

  //! \brief Run rectification
  //! \param cam0_dst Destination image to store the rectified camera 0 image in GPU memory.
  //! \param cam1_dst Destination image to store the rectified camera 1 image in GPU memory.
  //! \param cam0_src The source camera 0 image in GPU memory
  //! \param cam1_src The source camera 1 image in GPU memory
  void rectify(ImageGpu<Pixel>& cam0_dst,
               ImageGpu<Pixel>& cam1_dst,
               const ImageGpu<Pixel>& cam0_src,
               const ImageGpu<Pixel>& cam1_src) const;

  //! \brief Retrieves the computed undistortion-rectification maps
  //! \param cam_idx Camera index in (0, 1)
  const ImageGpu32fC2& getUndistortRectifyMap(int8_t cam_idx) const;

private:
  std::unique_ptr<StereoRectifier<CameraModel, DistortionModel, Pixel>> rectifiers_[2];
};

using HorizontalStereoPairRectifierEquidist32fC1 = HorizontalStereoPairRectifier<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
using HorizontalStereoPairRectifierRadTan32fC1 = HorizontalStereoPairRectifier<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
