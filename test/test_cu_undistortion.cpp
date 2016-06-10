#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_imgproc/cu_undistortion.cuh>

#include <ze/common/benchmark.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

using namespace ze;

TEST(impCuUndistortion, pinholeEquidistant_32fC1_Texture)
{
  using PinholeEquidistUndistorter = cu::ImageUndistorter<cu::PinholeGeometry, cu::EquidistantDistortion, Pixel32fC1>;

  const std::string test_data_name{"ze_feature_detection"};
  const std::string predefined_img_data_file_name{"752x480/pyr_0.png"};

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

  Eigen::RowVectorXf dist_coeff(4);
  dist_coeff << 0.00676530475436, -0.000811126898338, 0.0166458761987, -0.0172655346139;

  ze::cu::ImageGpu32fC1 gpu_src(*cv_img);
  ze::cu::ImageGpu32fC1 gpu_dst(cv_img->size());

  PinholeEquidistUndistorter undistorter(gpu_src.size(), cam_params, dist_coeff);
  undistorter.undistort(gpu_src, gpu_dst);  // GPU warm-up
  auto undistortLambda = [&](){
    undistorter.undistort(gpu_src, gpu_dst);
  };
  runTimingBenchmark(
        undistortLambda, 10, 20,
        "CUDA undistortion using Textures", true);
  ze::ImageCv32fC1 cv_img_out(gpu_dst);
  //! @todo (MPI) find a baseline and compare
  for (size_t y = 0; y < cv_img_out.height(); ++y)
  {
    for (size_t x = 0; x < cv_img_out.width(); ++x)
    {
      EXPECT_LE(cv_img_out.cvMat().at<float>(y, x), 1.0f);
      EXPECT_GE(cv_img_out.cvMat().at<float>(y, x), 0.0f);
    }
  }
}

ZE_UNITTEST_ENTRYPOINT
