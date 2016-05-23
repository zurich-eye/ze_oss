#include <imp/bridge/af/image_af.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <ze/common/file_utils.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

namespace ze {
constexpr const char* g_test_data_name =
    "ze_feature_detection";
constexpr const char* g_predefined_img_data_file_name =
    "752x480/pyr_0.png";
} // ze namespace

TEST(impBridgeAFTest, sum)
{
  std::string path(
        ze::joinPath(
          ze::getTestDataDir(ze::g_test_data_name),
          ze::g_predefined_img_data_file_name));
  ze::ImageCv32fC1::Ptr cv_img;
  ze::cvBridgeLoad(
        cv_img,
        path,
        ze::PixelOrder::gray);
  VLOG(2) << "loaded image " << path
          << ", size " << cv_img->size();
  ze::ImageAF32fC1 af_img(*cv_img);
  double af_sum = af::sum<float>(af_img.afArray());
  printf("AF sum: %f\n", af_sum);
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  printf("OpenCV sum: %f\n", cv_sum);
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

ZE_UNITTEST_ENTRYPOINT
