#include <imp/bridge/af/image_af.hpp>
#include <imp/bridge/af/pyramid_af.hpp>
#include <imp/bridge/af/fast_detector_af.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>

#include <ze/common/benchmark.h>
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

TEST(impBridgeAFTest, fastDetectorAF8uC1)
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
  ze::ImagePyramid8uC1::Ptr pyr =
      ze::createAFImagePyramid<ze::Pixel8uC1>(af_im, 0.5, 5, 8);

  uint32_t max_fts = 6000u;
  ze::Keypoints px_vec(2, max_fts);
  ze::KeypointScores score_vec(max_fts);
  ze::KeypointLevels level_vec(max_fts);
  ze::KeypointAngles angle_vec(max_fts);
  ze::KeypointTypes type_vec(max_fts);
  ze::Descriptors descriptors;
  uint32_t num_detected = 0u;
  ze::KeypointsWrapper features(
        px_vec, score_vec, level_vec, angle_vec, type_vec,
        descriptors, num_detected);
  ze::FastDetectorOptions fast_options;
  fast_options.threshold = 20.0f;
  ze::FastDetectorAF detector(fast_options, af_im->size());

  detector.detect(*pyr, features); // GPU warm-up
  auto detectLambda = [&](){
      features.num_detected = 0u; // Reset.
      detector.detect(*pyr, features);
    };
  ze::runTimingBenchmark(detectLambda, 10, 20, "AF FAST Detector", true);

#define ZE_TEST_FAST_AF_SHOW 0

#if ZE_TEST_FAST_AF_SHOW
  const int draw_len = 3;
#endif
  for (int8_t l=0; l<static_cast<int8_t>(pyr->numLevels()); ++l)
  {
    const auto& lvl = dynamic_cast<ze::ImageAF8uC1&>(pyr->at(l));
#if ZE_TEST_FAST_AF_SHOW
    af::array display_arr = af::colorSpace(lvl.afArray(), AF_RGB, AF_GRAY)/255.f;
#endif
    for (size_t f=0; f<features.num_detected; ++f)
    {
      if (l == features.levels(f))
      {
        const int x = features.px(0, f) * pyr->scaleFactor(l);
        const int y = features.px(1, f) * pyr->scaleFactor(l);
        EXPECT_LT(x, lvl.width());
        EXPECT_GT(x, 0);
        EXPECT_LT(y, lvl.height());
        EXPECT_GT(y, 0);
#if ZE_TEST_FAST_AF_SHOW
        display_arr(y, af::seq(x-draw_len, x+draw_len), 0) = 0.f;
        display_arr(y, af::seq(x-draw_len, x+draw_len), 1) = 1.f;
        display_arr(y, af::seq(x-draw_len, x+draw_len), 2) = 0.f;
        display_arr(af::seq(y-draw_len, y+draw_len), x, 0) = 0.f;
        display_arr(af::seq(y-draw_len, y+draw_len), x, 1) = 1.f;
        display_arr(af::seq(y-draw_len, y+draw_len), x, 2) = 0.f;
#endif
      }
    }
#if ZE_TEST_FAST_AF_SHOW
    af::Window wnd("AF array");
    while(!wnd.close())
    {
      wnd.image(display_arr);
    }
#endif
  }
}

ZE_UNITTEST_ENTRYPOINT
