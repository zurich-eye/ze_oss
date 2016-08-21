#include <imp/cu_imgproc/cu_horizontal_stereo_pair_rectifier.cuh>

namespace ze {
namespace cu {

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
HorizontalStereoPairRectifier<CameraModel, DistortionModel, Pixel>::HorizontalStereoPairRectifier(
    Vector4& transformed_cam0_params,
    Vector4& transformed_cam1_params,
    real_t& horizontal_offset,
    const Size2u& img_size,
    const Vector4& cam0_params,
    const Vector4& cam0_dist_coeffs,
    const Vector4& cam1_params,
    const Vector4& cam1_dist_coeffs,
    const Transformation& T_cam1_cam0)
{
  Matrix3 cam0_H;
  Matrix3 cam1_H;

  std::tie(cam0_H, cam1_H,
           transformed_cam0_params,
           transformed_cam1_params, horizontal_offset) =
      computeHorizontalStereoParameters <CameraModel, DistortionModel>(img_size,
                                                                       cam0_params,
                                                                       cam0_dist_coeffs,
                                                                       cam1_params,
                                                                       cam1_dist_coeffs,
                                                                       T_cam1_cam0);

  //! Allocate rectifiers
  const Matrix3 inv_cam0_H = cam0_H.inverse();
  const Matrix3 inv_cam1_H = cam1_H.inverse();
  rectifiers_[0].reset(
        new StereoRectifier<CameraModel, DistortionModel, Pixel>(
          img_size, cam0_params, transformed_cam0_params,
          cam0_dist_coeffs, inv_cam0_H));
  rectifiers_[1].reset(
        new StereoRectifier<CameraModel, DistortionModel, Pixel>(
          img_size, cam1_params, transformed_cam1_params,
          cam1_dist_coeffs, inv_cam1_H));
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
void HorizontalStereoPairRectifier<CameraModel, DistortionModel, Pixel>::rectify(
    ImageGpu<Pixel>& cam0_dst,
    ImageGpu<Pixel>& cam1_dst,
    const ImageGpu<Pixel>& cam0_src,
    const ImageGpu<Pixel>& cam1_src) const
{
  rectifiers_[0]->rectify(cam0_dst, cam0_src);
  rectifiers_[1]->rectify(cam1_dst, cam1_src);
}

template <typename CameraModel,
          typename DistortionModel,
          typename Pixel>
const ImageGpu32fC2& HorizontalStereoPairRectifier<CameraModel, DistortionModel, Pixel>::getUndistortRectifyMap(
    int8_t cam_idx) const
{
  CHECK_GE(cam_idx, 0);
  CHECK_LE(cam_idx, 1);
  return rectifiers_[cam_idx]->getUndistortRectifyMap();
}

// Explicit template instantiations
template class HorizontalStereoPairRectifier<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
template class HorizontalStereoPairRectifier<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
