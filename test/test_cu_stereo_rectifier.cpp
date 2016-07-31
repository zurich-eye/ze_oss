#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_imgproc/cu_stereo_rectification.cuh>

#include <ze/cameras/camera_rig.h>
#include <ze/common/benchmark.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

using namespace ze;

TEST(impCuStereoRectifierTexture, equidist32fC1)
{
  const double tolerance_on_sad{0.0014};   // Tolerance in SAD/nelems
  const double tolerance_on_sad_max{0.5};  // Tolerance on max of SAD

  const std::string test_folder =
      ze::joinPath(ze::getTestDataDir("imp_cu_imgproc"), "stereo_rectifier");
  const std::string calib_file =
      ze::joinPath(test_folder, "stereo_parameters.yaml");

  ze::CameraRig::Ptr rig = ze::cameraRigFromYaml(calib_file);
  VLOG(2) << "loaded camera rig from yaml file " << calib_file;

  // use 3x3 submatrix returned in left_P, right_P
  Eigen::Vector4f left_intrinsics;
  left_intrinsics << 972.2670826175212, 972.2670826175212, 654.0978851318359, 482.1916770935059;

  Eigen::Vector4f original_left_intrinsics =
      rig->at(0).projectionParameters().cast<float>();
  Eigen::Vector4f left_distortion =
      rig->at(0).distortionParameters().cast<float>();

  Eigen::Vector4f right_intrinsics;
  right_intrinsics << 972.2670826175212, 972.2670826175212, 654.0978851318359, 482.1916770935059;

  Eigen::Vector4f original_right_intrinsics =
      rig->at(1).projectionParameters().cast<float>();
  Eigen::Vector4f right_distortion =
      rig->at(1).distortionParameters().cast<float>();

  Eigen::Matrix3f left_H;
  left_H << 0.9999716948470486, 0.002684020348481424, -0.007028907417937792,
      -0.002697713727894102, 0.9999944805267602, -0.001939395951741348,
      0.007023663243873155, 0.00195830303687577, 0.9999734162485783;

  Eigen::Matrix3f right_H;
  right_H << 0.9999948321132365, 0.002852364833028826, -0.001483159357336634,
      -0.002849468937719842, 0.9999940370862417, 0.001950978916686848,
      0.001488715417037233, -0.001946742617730308, 0.9999969969552845;

  const std::string left_img_path = ze::joinPath(test_folder, "left01.png");
  ImageCv32fC1::Ptr cv_left_img;
  cvBridgeLoad(cv_left_img, left_img_path, PixelOrder::gray);
  VLOG(2) << "loaded image " << left_img_path
          << ", size " << cv_left_img->size();
  const std::string right_img_path = ze::joinPath(test_folder, "right01.png");
  ImageCv32fC1::Ptr cv_right_img;
  cvBridgeLoad(cv_right_img, right_img_path, PixelOrder::gray);
  VLOG(2) << "loaded image " << right_img_path
          << ", size " << cv_right_img->size();

  // Allocate GPU memory
  cu::ImageGpu32fC1 gpu_left_src(*cv_left_img);
  cu::ImageGpu32fC1 gpu_left_dst(cv_left_img->size());

  cu::ImageGpu32fC1 gpu_right_src(*cv_right_img);
  cu::ImageGpu32fC1 gpu_right_dst(cv_right_img->size());

  Eigen::Matrix3f left_H_inv = left_H.inverse();
  Eigen::Matrix3f right_H_inv = right_H.inverse();

  // Allocate rectifiers
  cu::RadTanStereoRectifier32fC1 left_rectifier(
        gpu_left_src.size(), left_intrinsics, original_left_intrinsics,
        left_distortion, left_H_inv);
  cu::RadTanStereoRectifier32fC1 right_rectifier(
        gpu_right_src.size(), right_intrinsics, original_right_intrinsics,
        right_distortion, right_H_inv);

  // Stereo rectify
  left_rectifier.rectify(gpu_left_dst, gpu_left_src);
  right_rectifier.rectify(gpu_right_dst, gpu_right_src);

  ImageCv32fC1 cv_left_img_out(gpu_left_dst);
  ImageCv32fC1 cv_right_img_out(gpu_right_dst);

  cv::imshow("CUDA_RECTIFIED_LEFT", cv_left_img_out.cvMat());
  cv::imshow("CUDA_RECTIFIED_RIGHT", cv_right_img_out.cvMat());
  cv::waitKey();
}

ZE_UNITTEST_ENTRYPOINT
