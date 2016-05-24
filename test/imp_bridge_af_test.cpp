#include <imp/bridge/af/image_af.hpp>
#include <imp/bridge/af/pyramid_af.hpp>
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

TEST(impBridgeAFTest, constructFromImpImage_32fC1)
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
  double af_sum = af::sum<double>(af_img.afArray());
  printf("AF sum: %f\n", af_sum);
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  printf("OpenCV sum: %f\n", cv_sum);
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

TEST(impBridgeAFTest, constructFromImpImage_8uC1)
{
  std::string path(
        ze::joinPath(
          ze::getTestDataDir(ze::g_test_data_name),
          ze::g_predefined_img_data_file_name));

  ze::ImageCv8uC1::Ptr cv_img;
  ze::cvBridgeLoad(
        cv_img,
        path,
        ze::PixelOrder::gray);
  VLOG(2) << "loaded image " << path
          << ", size " << cv_img->size();
  ze::ImageAF8uC1 af_img(*cv_img);
  double af_sum = af::sum<double>(af_img.afArray());
  printf("AF sum: %f\n", af_sum);
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  printf("OpenCV sum: %f\n", cv_sum);
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

TEST(impBridgeAFTest, constructFromAFArray_32fC1)
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

  std::unique_ptr<float[]> h_buffer(
        new float[cv_img->width()*cv_img->height()]);
  for(size_t r=0; r < cv_img->height(); ++r)
  {
    for(size_t c=0; c < cv_img->width(); ++c)
    {
      h_buffer.get()[c*cv_img->height()+r] = cv_img->pixel(c, r);
    }
  }
  ze::ImageAF32fC1 af_img(
        af::array(
          cv_img->height(),
          cv_img->width(),
          h_buffer.get()));
  double af_sum = af::sum<double>(af_img.afArray());
  printf("AF sum: %f\n", af_sum);
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  printf("OpenCV sum: %f\n", cv_sum);
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

TEST(impBridgeAFTest, constructFromAFArray_8uC1)
{
  std::string path(
        ze::joinPath(
          ze::getTestDataDir(ze::g_test_data_name),
          ze::g_predefined_img_data_file_name));

  ze::ImageCv8uC1::Ptr cv_img;
  ze::cvBridgeLoad(
        cv_img,
        path,
        ze::PixelOrder::gray);

  std::unique_ptr<unsigned char[]> h_buffer(
        new unsigned char[cv_img->width()*cv_img->height()]);
  for(size_t r=0; r < cv_img->height(); ++r)
  {
    for(size_t c=0; c < cv_img->width(); ++c)
    {
      h_buffer.get()[c*cv_img->height()+r] = cv_img->pixel(c, r);
    }
  }
  ze::ImageAF8uC1 af_img(
        af::array(
          cv_img->height(),
          cv_img->width(),
          h_buffer.get()));
  double af_sum = af::sum<double>(af_img.afArray());
  printf("AF sum: %f\n", af_sum);
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  printf("OpenCV sum: %f\n", cv_sum);
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

TEST(impBridgeAFTest, createAFImagePyramid8uC1)
{
  std::string path(
        ze::joinPath(
          ze::getTestDataDir(ze::g_test_data_name),
          ze::g_predefined_img_data_file_name));
  ze::ImageCv8uC1::Ptr cv_img;
  ze::cvBridgeLoad(
        cv_img,
        path,
        ze::PixelOrder::gray);

  // Create AF pyramid
  ze::ImageAF8uC1::Ptr af_im =
      std::make_shared<ze::ImageAF8uC1>(*cv_img);
  ze::ImagePyramid8uC1::Ptr af_pyr =
      ze::createAFImagePyramid<ze::Pixel8uC1>(af_im, 0.5, 5, 8);
  // Create IMP pyramid
  auto pyr = ze::createImagePyramidCpu<ze::Pixel8uC1>(cv_img, 0.5, 5, 8);

  // Compare
  for (size_t l=0; l<pyr->numLevels(); ++l)
  {
    ze::ImageAF8uC1 af_img_l(pyr->at(l));
    // Compare AF arrays
    const auto& lvl = dynamic_cast<ze::ImageAF8uC1&>(af_pyr->at(l));
    af::array test = lvl.afArray() / 255.0f;
    af::array gt = af_img_l.afArray() / 255.0f;
    double sad = af::sum<double>(af::abs(test - gt));
    double nel = lvl.numel();
    printf("level %lu SAD: %f, SAD/nel: %f\n", l, sad, sad/nel);
    EXPECT_LT(sad/nel, 0.1);
  }
}

ZE_UNITTEST_ENTRYPOINT
