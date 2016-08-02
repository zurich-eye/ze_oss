#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_imgproc/horizontal_stereo_pair_rectifier.hpp>

#include <ze/cameras/camera_rig.h>
#include <ze/common/benchmark.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

using namespace ze;

TEST(impCuStereoRectifierTexture, equidist32fC1)
{
  constexpr float c_map_tolearance{0.001};
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

  Eigen::Matrix3f left_H;
  left_H << 0.9999716948470486, 0.002684020348481424, -0.007028907417937792,
      -0.002697713727894102, 0.9999944805267602, -0.001939395951741348,
      0.007023663243873155, 0.00195830303687577, 0.9999734162485783;

  const std::string left_img_path = ze::joinPath(test_folder, "left01.png");
  ImageCv32fC1::Ptr cv_left_img;
  cvBridgeLoad(cv_left_img, left_img_path, PixelOrder::gray);
  VLOG(2) << "loaded image " << left_img_path
          << ", size " << cv_left_img->size();

  const size_t img_width = cv_left_img->width();
  const size_t img_height = cv_left_img->height();
  const size_t img_n_elems = img_width * img_height;

  Eigen::Matrix3f left_H_inv = left_H.inverse();

  // Allocate rectifier
  cu::RadTanStereoRectifier32fC1 left_rectifier(
        cv_left_img->size(), left_intrinsics, original_left_intrinsics,
        left_distortion, left_H_inv);

  ImageCv32fC2 left_map(left_rectifier.getUndistortRectifyMap());
  CHECK_EQ(cv_left_img->size(), left_map.size());

  // Read ground-truth maps
  const std::string gt_left_map_x_path =
      joinPath(test_folder, "map_x_left01.bin");
  const std::string gt_left_map_y_path =
      joinPath(test_folder, "map_y_left01.bin");

  CHECK(fileExists(gt_left_map_x_path));
  CHECK(fileExists(gt_left_map_y_path));

  std::ifstream gt_left_map_x_file(gt_left_map_x_path, std::ofstream::binary);
  std::ifstream gt_left_map_y_file(gt_left_map_y_path, std::ofstream::binary);
  CHECK(gt_left_map_x_file.is_open());
  CHECK(gt_left_map_y_file.is_open());

  std::unique_ptr<float[]> gt_map_x(new float[img_n_elems]);
  std::unique_ptr<float[]> gt_map_y(new float[img_n_elems]);

  gt_left_map_x_file.read(
        reinterpret_cast<char*>(gt_map_x.get()), img_n_elems*sizeof(float));
  gt_left_map_y_file.read(
        reinterpret_cast<char*>(gt_map_y.get()), img_n_elems*sizeof(float));

  // Compare computed map with ground-truth map
  for (uint32_t y = 0; y < img_height; ++y)
  {
    for (uint32_t x = 0; x < img_width; ++x)
    {
      EXPECT_NEAR(
            gt_map_x.get()[y*img_width+x], left_map(x, y)[0], c_map_tolearance);
      EXPECT_NEAR(
            gt_map_y.get()[y*img_width+x], left_map(x, y)[1], c_map_tolearance);
    }
  }
}

TEST(impCuStereoRectifierTexture, horizontalStereoPairEquidist32fC1)
{
  constexpr float c_map_tolearance{0.001};
  const std::string test_folder =
      ze::joinPath(ze::getTestDataDir("imp_cu_imgproc"), "stereo_rectifier");
  const std::string calib_file =
      ze::joinPath(test_folder, "stereo_parameters.yaml");

  ze::CameraRig::Ptr rig = ze::cameraRigFromYaml(calib_file);
  VLOG(2) << "loaded camera rig from yaml file " << calib_file;

  Eigen::Vector4f left_intrinsics =
      rig->at(0).projectionParameters().cast<float>();
  Eigen::Vector4f left_distortion =
      rig->at(0).distortionParameters().cast<float>();

  const std::string left_img_path = ze::joinPath(test_folder, "left01.png");
  ImageCv32fC1::Ptr cv_left_img;
  cvBridgeLoad(cv_left_img, left_img_path, PixelOrder::gray);
  VLOG(2) << "loaded image " << left_img_path
          << ", size " << cv_left_img->size();

  Eigen::Vector4f right_intrinsics =
      rig->at(1).projectionParameters().cast<float>();
  Eigen::Vector4f right_distortion =
      rig->at(1).distortionParameters().cast<float>();

  const std::string right_img_path = ze::joinPath(test_folder, "right01.png");
  ImageCv32fC1::Ptr cv_right_img;
  cvBridgeLoad(cv_right_img, right_img_path, PixelOrder::gray);
  VLOG(2) << "loaded image " << right_img_path
          << ", size " << cv_right_img->size();

  const size_t img_width = cv_left_img->width();
  const size_t img_height = cv_left_img->height();
  const size_t img_n_elems = img_width * img_height;

  ze::Transformation T_C0_B = rig->T_C_B(0);
  ze::Transformation T_C1_B = rig->T_C_B(1);
  ze::Transformation T_C0_C1 = T_C0_B * T_C1_B.inverse();
  // Allocate rectifier
  ze::Position t = T_C0_C1.getPosition();
  Eigen::Vector3f t_l_r(t(0), t(1), t(2));
  cu::HorizontalStereoPairRectifierEquidist32fC1 rectifier(
        cv_left_img->size(),
        left_intrinsics,
        left_distortion,
        right_intrinsics,
        right_distortion,
        T_C0_C1.getRotationMatrix(),
        t_l_r);
}

ZE_UNITTEST_ENTRYPOINT
