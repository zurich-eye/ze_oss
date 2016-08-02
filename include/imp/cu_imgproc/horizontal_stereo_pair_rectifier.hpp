#pragma once

#include <imp/cu_imgproc/cu_stereo_rectification.cuh>

namespace ze {
namespace cu {

template<typename CameraModel,
         typename DistortionModel,
         typename Pixel>
class HorizontalStereoPairRectifier
{
public:
  HorizontalStereoPairRectifier(Size2u img_size,
                                Eigen::Vector4f& left_camera_params,
                                Eigen::Vector4f& left_dist_coeffs,
                                Eigen::Vector4f& right_camera_params,
                                Eigen::Vector4f& right_dist_coeffs,
                                Eigen::Matrix3f& R_l_r,
                                Eigen::Vector3f& t_l_r);

  ~HorizontalStereoPairRectifier() = default;

  void rectify(ImageGpu<Pixel>& left_dst,
               ImageGpu<Pixel>& right_dst,
               const ImageGpu<Pixel>& left_src,
               const ImageGpu<Pixel>& right_src) const;
private:
  std::unique_ptr<StereoRectifier<CameraModel, DistortionModel, Pixel>> left_rectifier_;
  std::unique_ptr<StereoRectifier<CameraModel, DistortionModel, Pixel>> right_rectifier_;
};

using HorizontalStereoPairRectifierEquidist32fC1 = HorizontalStereoPairRectifier<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
using HorizontalStereoPairRectifierRadTan32fC1 = HorizontalStereoPairRectifier<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;


} // cu namespace
} // ze namespace
