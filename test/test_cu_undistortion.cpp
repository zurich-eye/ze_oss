#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_imgproc/cu_undistortion.cuh>

#include <ze/common/benchmark.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

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

  Eigen::RowVectorXf cam_params(4);
  cam_params << 471.690643292, 471.765601046, 371.087464172, 228.63874151;
  Eigen::RowVectorXf dist_coeffs(4);
  dist_coeffs << 0, 0, 0, 0;

  cu::ImageGpu32fC1 gpu_src(*cv_img);
  cu::ImageGpu32fC1 gpu_dst(cv_img->size());

  cu::RadTanUndistort32fC1 undistorter(
        gpu_src.size(), cam_params, dist_coeffs);
  undistorter.undistort(gpu_src, gpu_dst);  // GPU warm-up
  auto undistortLambda = [&](){
    undistorter.undistort(gpu_src, gpu_dst);
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

  Eigen::RowVectorXf cam_params(4);
  cam_params << 471.690643292, 471.765601046, 371.087464172, 228.63874151;
  Eigen::RowVectorXf dist_coeffs(4);
  dist_coeffs << 0.00676530475436, -0.000811126898338, 0.0166458761987, -0.0172655346139;

  cu::EquidistUndistort32fC1 undistorter(
        cv_img->size(), cam_params, dist_coeffs);

  ImageCv32fC2 cv_computed_map(undistorter.getUndistortionMap());
  for (uint32_t y = 0; y < cv_computed_map.height(); ++y)
  {
    for (uint32_t x = 0; x < cv_computed_map.width(); ++x)
    {
      float px[2];
      px[0] = x;
      px[1] = y;
      cu::PinholeGeometry::backProject(cam_params.data(), px);
      cu::EquidistantDistortion::distort(dist_coeffs.data(), px);
      cu::PinholeGeometry::project(cam_params.data(), px);
      EXPECT_NEAR(px[0], cv_computed_map(x, y)[0], 0.0005);
    }
  }
}

TEST(impCuUndistortionTexture, equidist32fC1)
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

  Eigen::RowVectorXf cam_params(4);
  cam_params << 471.690643292, 471.765601046, 371.087464172, 228.63874151;
  Eigen::RowVectorXf dist_coeffs(4);
  dist_coeffs << 0.00676530475436, -0.000811126898338, 0.0166458761987, -0.0172655346139;

  cu::ImageGpu32fC1 gpu_src(*cv_img);
  cu::ImageGpu32fC1 gpu_dst(cv_img->size());

  cu::EquidistUndistort32fC1 undistorter(
        gpu_src.size(), cam_params, dist_coeffs);
  undistorter.undistort(gpu_src, gpu_dst);  // GPU warm-up
  auto undistortLambda = [&](){
    undistorter.undistort(gpu_src, gpu_dst);
  };
  runTimingBenchmark(
        undistortLambda, 10, 20,
        "CUDA undistortion using Textures", true);
}

ZE_UNITTEST_ENTRYPOINT
