#include <imp/bridge/opencv/cv_bridge.hpp>
#include <ze/cameras/camera_rig.h>
#include <ze/common/benchmark.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/geometry/epipolar_geometry.hpp>

int main(int argc, char** argv)
{
  using namespace ze;
  const std::string test_folder =
      ze::joinPath(ze::getTestDataDir("imp_cu_imgproc"), "stereo_rectifier");
  const std::string calib_file =
      ze::joinPath(test_folder, "stereo_parameters.yaml");

  ze::CameraRig::Ptr rig = ze::cameraRigFromYaml(calib_file);
  VLOG(2) << "loaded camera rig from yaml file " << calib_file;

  Vector4 left_cam_params =
      rig->at(0).projectionParameters().cast<FloatType>();
  Vector4 left_distortion =
      rig->at(0).distortionParameters().cast<FloatType>();

  const std::string left_img_path = ze::joinPath(test_folder, "left01.png");
  ImageCv32fC1::Ptr cv_left_img;
  cvBridgeLoad(cv_left_img, left_img_path, PixelOrder::gray);
  VLOG(2) << "loaded image " << left_img_path
          << ", size " << cv_left_img->size();

  Vector4 right_cam_params =
      rig->at(1).projectionParameters().cast<FloatType>();
  Vector4 right_distortion =
      rig->at(1).distortionParameters().cast<FloatType>();

  const std::string right_img_path = ze::joinPath(test_folder, "right01.png");
  ImageCv32fC1::Ptr cv_right_img;
  cvBridgeLoad(cv_right_img, right_img_path, PixelOrder::gray);
  VLOG(2) << "loaded image " << right_img_path
          << ", size " << cv_right_img->size();

  Transformation T_C0_B = rig->T_C_B(0);
  Transformation T_C1_B = rig->T_C_B(1);
  Transformation T_L_R = T_C0_B * T_C1_B.inverse();


  Vector4 transformed_left_cam_params;
  Vector4 transformed_right_cam_params;
  float horizontal_offset;

  Matrix3 left_H;
  Matrix3 right_H;

  computeHorizontalStereoParameters(cv_left_img->size(),
                                    left_cam_params,
                                    left_distortion,
                                    right_cam_params,
                                    right_distortion,
                                    T_L_R,
                                    left_H,
                                    right_H,
                                    transformed_left_cam_params,
                                    transformed_right_cam_params,
                                    horizontal_offset);
}
