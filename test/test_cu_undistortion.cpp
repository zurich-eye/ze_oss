#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_imgproc/cu_undistortion.cuh>

#include <ze/cameras/camera_rig.hpp>
#include <ze/common/benchmark.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>

using namespace ze;

TEST(impCuUndistortionTexture, radTan32fC1_zeroDistortion)
{
  const std::string test_data_name{"imp_cu_imgproc"};
  const std::string predefined_img_data_file_name{"pyr_0.png"};

  std::string path(
        joinPath(
          getTestDataDir(test_data_name),
          predefined_img_data_file_name));

  ImageCv32fC1::Ptr cv_img;
  cvBridgeLoad(cv_img, path, PixelOrder::gray);
  VLOG(2) << "loaded image " << path
          << ", size " << cv_img->size();

  VectorX cam_params(4);
  cam_params << 471.690643292, 471.765601046, 371.087464172, 228.63874151;
  VectorX dist_coeffs(4);
  dist_coeffs << 0.0, 0.0, 0.0, 0.0;

  cu::ImageGpu32fC1 gpu_src(*cv_img);
  cu::ImageGpu32fC1 gpu_dst(cv_img->size());

  cu::RadTanUndistort32fC1 undistorter(
        gpu_src.size(), cam_params, dist_coeffs);
  undistorter.undistort(gpu_dst, gpu_src);  // GPU warm-up
  auto undistortLambda = [&](){
    undistorter.undistort(gpu_dst, gpu_src);
  };
  runTimingBenchmark(
        undistortLambda, 10, 20,
        "CUDA undistortion using Textures", true);

  ImageCv32fC1 cv_img_out(gpu_dst);
  for (uint32_t y = 0; y < cv_img_out.height(); ++y)
  {
    for (uint32_t x = 0; x < cv_img_out.width(); ++x)
    {
      EXPECT_FLOAT_EQ(cv_img->cvMat().at<float>(y, x),
                      cv_img_out.cvMat().at<float>(y, x));
    }
  }
}

TEST(impCuUndistortionTexture, equidist32fC1_testMap)
{
  const std::string test_data_name{"imp_cu_imgproc"};
  const std::string predefined_img_data_file_name{"pyr_0.png"};

  std::string path(
        joinPath(
          getTestDataDir(test_data_name),
          predefined_img_data_file_name));

  ImageCv32fC1::Ptr cv_img;
  cvBridgeLoad(cv_img, path, PixelOrder::gray);
  VLOG(2) << "loaded image " << path
          << ", size " << cv_img->size();

  VectorX cam_params(4);
  cam_params << 471.690643292, 471.765601046, 371.087464172, 228.63874151;
  VectorX dist_coeffs(4);
  dist_coeffs << 0.00676530475436, -0.000811126898338, 0.0166458761987, -0.0172655346139;

  cu::EquidistUndistort32fC1 undistorter(
        cv_img->size(), cam_params, dist_coeffs);

  // Copute map on CPU
  Eigen::VectorXf cp_flt = cam_params.cast<float>();
  Eigen::VectorXf dist_flt = dist_coeffs.cast<float>();
  ImageCv32fC2 cv_computed_map(undistorter.getUndistortionMap());
  for (uint32_t y = 0; y < cv_computed_map.height(); ++y)
  {
    for (uint32_t x = 0; x < cv_computed_map.width(); ++x)
    {
      float px[2];
      px[0] = x;
      px[1] = y;
      PinholeGeometry::backProject(cp_flt.data(), px);
      EquidistantDistortion::distort(dist_flt.data(), px);
      PinholeGeometry::project(cp_flt.data(), px);
      EXPECT_NEAR(px[0], cv_computed_map(x, y)[0], 0.0005);
    }
  }
}

TEST(impCuUndistortionTexture, equidist32fC1)
{
  const double tolerance_on_sad{0.0014};   // Tolerance in SAD/nelems
  const double tolerance_on_sad_max{0.5};  // Tolerance on max of SAD
  const std::string test_folder =
      ze::getTestDataDir("imp_cu_imgproc");
  const std::string calib_file =
      ze::joinPath(test_folder, "visensor_22030_swe_params.yaml");

  ze::CameraRig::Ptr rig = ze::cameraRigFromYaml(calib_file);
  VLOG(2) << "loaded camera rig from yaml file " << calib_file;

  for (int i = 0; i < 12; ++i)
  {
    for (int lr = 0; lr < 2; ++lr)
    {
      std::stringstream file_suffix;
      file_suffix << lr << "_" << i << ".png";
      const std::string dist_img_file =
          ze::joinPath(test_folder, "distorted" + file_suffix.str());
      const std::string undist_img_file =
          ze::joinPath(test_folder, "undistorted" + file_suffix.str());
      VLOG(2) << "undistorted GT file: " << undist_img_file;
      VLOG(2) << "distorted file: " << dist_img_file;
      // Load test image
      ImageCv32fC1::Ptr cv_img;
      cvBridgeLoad(cv_img, dist_img_file, PixelOrder::gray);
      VLOG(2) << "loaded image " << dist_img_file
              << ", size " << cv_img->size();
      // Allocate GPU memory
      cu::ImageGpu32fC1 gpu_src(*cv_img);
      cu::ImageGpu32fC1 gpu_dst(cv_img->size());

      // Camera parameters
      VectorX cam_params = rig->at(lr).projectionParameters();
      VectorX dist_coeffs = rig->at(lr).distortionParameters();

      cu::EquidistUndistort32fC1 undistorter(
            gpu_src.size(), cam_params, dist_coeffs);
      undistorter.undistort(gpu_dst, gpu_src);  // GPU warm-up
      auto undistortLambda = [&](){
        undistorter.undistort(gpu_dst, gpu_src);
      };
      runTimingBenchmark(
            undistortLambda, 10, 20,
            "CUDA undistortion using Textures", true);

      // Download result image
      ImageCv32fC1 cv_img_out(gpu_dst);

      // Load GT undistorted image
      cvBridgeLoad(cv_img, undist_img_file, PixelOrder::gray);

      // Compare
      cv::Mat abs_diff = cv::abs(cv_img->cvMat() - cv_img_out.cvMat());
      double ad_min, ad_max;

      cv::minMaxLoc(abs_diff, &ad_min, &ad_max);
      EXPECT_LT(ad_max, tolerance_on_sad_max);

      double sad = cv::sum(abs_diff)[0];
      EXPECT_LT(sad/static_cast<double>(cv_img->numel()), tolerance_on_sad)
          << " - testing image " << dist_img_file ;
    }
  }
}

ZE_UNITTEST_ENTRYPOINT
