#include <imp/cu_imgproc/cu_horizontal_stereo_pair_rectifier.cuh>

namespace ze {
namespace cu {

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
HorizontalStereoPairRectifier<CameraModel, DistortionModel, Pixel>::HorizontalStereoPairRectifier(
    Vector4& transformed_left_cam_params,
    Vector4& transformed_right_cam_params,
    FloatType& horizontal_offset,
    const Size2u& img_size,
    const Vector4& left_camera_params,
    const Vector4& left_dist_coeffs,
    const Vector4& right_camera_params,
    const Vector4& right_dist_coeffs,
    const Transformation& T_L_R)
{
  Matrix3 left_H;
  Matrix3 right_H;

  std::tie(left_H, right_H,
           transformed_left_cam_params,
           transformed_right_cam_params, horizontal_offset) =
      computeHorizontalStereoParameters <CameraModel, DistortionModel>(img_size,
                                                                       left_camera_params,
                                                                       left_dist_coeffs,
                                                                       right_camera_params,
                                                                       right_dist_coeffs,
                                                                       T_L_R);

  //! Allocate rectifiers for the left and right cameras
  const Matrix3 inv_left_H = left_H.inverse();
  const Matrix3 inv_right_H = right_H.inverse();
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
