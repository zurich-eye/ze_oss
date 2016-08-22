#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_imgproc/cu_stereo_rectification.cuh>
#include <imp/cu_imgproc/cu_horizontal_stereo_pair_rectifier.cuh>
#include <ze/cameras/camera_rig.h>
#include <ze/common/benchmark.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

namespace ze {

void testRectificationMapAgainstFile(
    const ImageCv32fC2& map,
    const std::string& map_x_file_path,
    const std::string& map_y_file_path,
    float tolerance)
{
  CHECK(fileExists(map_x_file_path));
  CHECK(fileExists(map_y_file_path));

  std::ifstream map_x_file(map_x_file_path, std::ofstream::binary);
  std::ifstream map_y_file(map_y_file_path, std::ofstream::binary);
  CHECK(map_x_file.is_open());
  CHECK(map_y_file.is_open());

  const size_t map_width = map.width();
  const size_t map_height = map.height();
  const size_t map_n_elems = map_width * map_height;

  std::unique_ptr<float[]> gt_map_x(new float[map_n_elems]);
  std::unique_ptr<float[]> gt_map_y(new float[map_n_elems]);

  map_x_file.read(
        reinterpret_cast<char*>(gt_map_x.get()), map_n_elems*sizeof(float));
  map_y_file.read(
        reinterpret_cast<char*>(gt_map_y.get()), map_n_elems*sizeof(float));

  // Compare computed map with ground-truth map
  for (uint32_t y = 0; y < map_height; ++y)
  {
    for (uint32_t x = 0; x < map_width; ++x)
    {
      EXPECT_NEAR(
            gt_map_x.get()[y*map_width+x], map(x, y)[0], tolerance);
      EXPECT_NEAR(
            gt_map_y.get()[y*map_width+x], map(x, y)[1], tolerance);
    }
  }
}

} // ze namespace

using namespace ze;

TEST(impCuStereoRectifierTexture, radTan32fC1)
{
  constexpr float c_map_tolearance{0.001};
  const std::string test_folder =
      ze::joinPath(ze::getTestDataDir("imp_cu_imgproc"), "stereo_rectifier");
  const std::string calib_file =
      ze::joinPath(test_folder, "stereo_parameters.yaml");

  ze::CameraRig::Ptr rig = ze::cameraRigFromYaml(calib_file);
  VLOG(2) << "loaded camera rig from yaml file " << calib_file;

  // use 3x3 submatrix returned in left_P, right_P
  Vector4 left_intrinsics;
  left_intrinsics << 972.2670826175212, 972.2670826175212, 654.0978851318359, 482.1916770935059;

  Vector4 original_left_intrinsics =
      rig->at(0).projectionParameters();
  Vector4 left_distortion =
      rig->at(0).distortionParameters();

  Matrix3 left_H;
  left_H << 0.9999716948470486, 0.002684020348481424, -0.007028907417937792,
      -0.002697713727894102, 0.9999944805267602, -0.001939395951741348,
      0.007023663243873155, 0.00195830303687577, 0.9999734162485783;

  const std::string left_img_path = ze::joinPath(test_folder, "left01.png");
  ImageCv32fC1::Ptr cv_left_img;
  cvBridgeLoad(cv_left_img, left_img_path, PixelOrder::gray);
  VLOG(2) << "loaded image " << left_img_path
          << ", size " << cv_left_img->size();

  Matrix3 left_H_inv = left_H.inverse();

  // Allocate rectifier
  cu::RadTanStereoRectifier32fC1 left_rectifier(
        cv_left_img->size(), original_left_intrinsics, left_intrinsics,
        left_distortion, left_H_inv);

  ImageCv32fC2 left_map(left_rectifier.getUndistortRectifyMap());
  CHECK_EQ(cv_left_img->size(), left_map.size());

  // Test against ground-truth maps
  const std::string gt_left_map_x_path =
      joinPath(test_folder, "map_x_left.bin");
  const std::string gt_left_map_y_path =
      joinPath(test_folder, "map_y_left.bin");
  testRectificationMapAgainstFile(left_map,
                                  gt_left_map_x_path,
                                  gt_left_map_y_path,
                                  c_map_tolearance);
}

TEST(impCuStereoRectifierTexture, horizontalStereoPairRadTan32fC1)
{
  constexpr float c_map_tolearance{1.0f};
  const std::string test_folder =
      ze::joinPath(ze::getTestDataDir("imp_cu_imgproc"), "stereo_rectifier");
  const std::string calib_file =
      ze::joinPath(test_folder, "stereo_parameters.yaml");

  ze::CameraRig::Ptr rig = ze::cameraRigFromYaml(calib_file);
  VLOG(2) << "loaded camera rig from yaml file " << calib_file;

  Vector4 left_cam_params =
      rig->at(0).projectionParameters();
  Vector4 left_distortion =
      rig->at(0).distortionParameters();

  const std::string left_img_path = ze::joinPath(test_folder, "left01.png");
  ImageCv32fC1::Ptr cv_left_img;
  cvBridgeLoad(cv_left_img, left_img_path, PixelOrder::gray);
  VLOG(2) << "loaded image " << left_img_path
          << ", size " << cv_left_img->size();

  Vector4 right_cam_params =
      rig->at(1).projectionParameters();
  Vector4 right_distortion =
      rig->at(1).distortionParameters();

  const std::string right_img_path = ze::joinPath(test_folder, "right01.png");
  ImageCv32fC1::Ptr cv_right_img;
  cvBridgeLoad(cv_right_img, right_img_path, PixelOrder::gray);
  VLOG(2) << "loaded image " << right_img_path
          << ", size " << cv_right_img->size();

  ze::Transformation T_C0_B = rig->T_C_B(0);
  ze::Transformation T_C1_B = rig->T_C_B(1);
  ze::Transformation T_C0_C1 = T_C0_B * T_C1_B.inverse();

  VLOG(2) << "Stereo extrinsics (T_C0_C1):\n" << T_C0_C1;
  VLOG(2) << "Stereo extrinsics (T_C1_C0):\n" << T_C0_C1.inverse();

  // Allocate rectifier
  Vector4 transformed_left_cam_params;
  Vector4 transformed_right_cam_params;
  FloatType horizontal_offset;
  cu::HorizontalStereoPairRectifierRadTan32fC1 rectifier(
        transformed_left_cam_params,
        transformed_right_cam_params,
        horizontal_offset,
        cv_left_img->size(),
        left_cam_params,
        left_distortion,
        right_cam_params,
        right_distortion,
        T_C0_C1);

  // Download maps from GPU
  ImageCv32fC2 left_map(rectifier.getUndistortRectifyMap(0));
  CHECK_EQ(cv_left_img->size(), left_map.size());
  ImageCv32fC2 right_map(rectifier.getUndistortRectifyMap(1));
  CHECK_EQ(cv_right_img->size(), right_map.size());

  // Test against ground-truth maps
  const std::string gt_left_map_x_path =
      joinPath(test_folder, "map_x_left.bin");
  const std::string gt_left_map_y_path =
      joinPath(test_folder, "map_y_left.bin");
  const std::string gt_right_map_x_path =
      joinPath(test_folder, "map_x_right.bin");
  const std::string gt_right_map_y_path =
      joinPath(test_folder, "map_y_right.bin");

  testRectificationMapAgainstFile(left_map,
                                  gt_left_map_x_path,
                                  gt_left_map_y_path,
                                  c_map_tolearance);
  testRectificationMapAgainstFile(right_map,
                                  gt_right_map_x_path,
                                  gt_right_map_y_path,
                                  c_map_tolearance);
}

ZE_UNITTEST_ENTRYPOINT
