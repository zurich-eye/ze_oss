#include <imp/bridge/af/fast_detector_af.hpp>
#include <imp/bridge/af/orb_detector_af.hpp>
#include <imp/bridge/af/sift_detector_af.hpp>

#include <imp/bridge/opencv/cv_bridge.hpp>

#include <ze/common/benchmark.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

namespace ze {
const std::string g_test_data_name{"ze_feature_detection"};
const std::string g_predefined_img_data_file_name{"752x480/pyr_0.png"};
} // ze namespace

using namespace ze;

TEST(impBridgeAFTest, constructFromImpImage_32fC1)
{
  std::string path(
        joinPath(
          getTestDataDir(g_test_data_name),
          g_predefined_img_data_file_name));

  ImageCv32fC1::Ptr cv_img;
  cvBridgeLoad(
        cv_img,
        path,
        PixelOrder::gray);
  VLOG(2) << "loaded image " << path
          << ", size " << cv_img->size();
  ImageAF32fC1 af_img(*cv_img);
  double af_sum = af::sum<double>(af_img.afArray());
  printf("AF sum: %f\n", af_sum);
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  printf("OpenCV sum: %f\n", cv_sum);
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

TEST(impBridgeAFTest, constructFromImpImage_8uC1)
{
  std::string path(
        joinPath(
          getTestDataDir(g_test_data_name),
          g_predefined_img_data_file_name));

  ImageCv8uC1::Ptr cv_img;
  cvBridgeLoad(
        cv_img,
        path,
        PixelOrder::gray);
  VLOG(2) << "loaded image " << path
          << ", size " << cv_img->size();
  ImageAF8uC1 af_img(*cv_img);
  double af_sum = af::sum<double>(af_img.afArray());
  printf("AF sum: %f\n", af_sum);
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  printf("OpenCV sum: %f\n", cv_sum);
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

TEST(impBridgeAFTest, constructFromAFArray_32fC1)
{
  std::string path(
        joinPath(
          getTestDataDir(g_test_data_name),
          g_predefined_img_data_file_name));

  ImageCv32fC1::Ptr cv_img;
  cvBridgeLoad(
        cv_img,
        path,
        PixelOrder::gray);

  std::unique_ptr<float[]> h_buffer(
        new float[cv_img->width()*cv_img->height()]);
  for(size_t r=0; r < cv_img->height(); ++r)
  {
    for(size_t c=0; c < cv_img->width(); ++c)
    {
      h_buffer.get()[c*cv_img->height()+r] = cv_img->pixel(c, r);
    }
  }
  ImageAF32fC1 af_img(
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
        joinPath(
          getTestDataDir(g_test_data_name),
          g_predefined_img_data_file_name));

  ImageCv8uC1::Ptr cv_img;
  cvBridgeLoad(
        cv_img,
        path,
        PixelOrder::gray);

  std::unique_ptr<unsigned char[]> h_buffer(
        new unsigned char[cv_img->width()*cv_img->height()]);
  for(size_t r=0; r < cv_img->height(); ++r)
  {
    for(size_t c=0; c < cv_img->width(); ++c)
    {
      h_buffer.get()[c*cv_img->height()+r] = cv_img->pixel(c, r);
    }
  }
  ImageAF8uC1 af_img(
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
        joinPath(
          getTestDataDir(g_test_data_name),
          g_predefined_img_data_file_name));
  ImageCv8uC1::Ptr cv_img;
  cvBridgeLoad(
        cv_img,
        path,
        PixelOrder::gray);

  // Create AF pyramid
  ImageAF8uC1::Ptr af_im =
      std::make_shared<ImageAF8uC1>(*cv_img);
  ImagePyramid8uC1::Ptr af_pyr =
      createAFImagePyramid<Pixel8uC1>(af_im, 0.5, 5, 8);
  // Create IMP pyramid
  auto pyr = createImagePyramidCpu<Pixel8uC1>(cv_img, 0.5, 5, 8);

  // Compare
  for (size_t l=0; l<pyr->numLevels(); ++l)
  {
    ImageAF8uC1 af_img_l(pyr->at(l));
    // Compare AF arrays
    const auto& lvl = dynamic_cast<ImageAF8uC1&>(af_pyr->at(l));
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
        joinPath(
          getTestDataDir(g_test_data_name),
          g_predefined_img_data_file_name));
  ImageCv8uC1::Ptr cv_img;
  cvBridgeLoad(
        cv_img,
        path,
        PixelOrder::gray);

  // Create AF pyramid
  ImageAF8uC1::Ptr af_im =
      std::make_shared<ImageAF8uC1>(*cv_img);
  ImagePyramid8uC1::Ptr pyr =
      createAFImagePyramid<Pixel8uC1>(af_im, 0.5, 5, 8);

  uint32_t max_fts = 6000u;
  Keypoints px_vec(2, max_fts);
  KeypointScores score_vec(max_fts);
  KeypointLevels level_vec(max_fts);
  KeypointAngles angle_vec(max_fts);
  KeypointTypes type_vec(max_fts);
  Descriptors descriptors;
  uint32_t num_detected = 0u;
  KeypointsWrapper features(
        px_vec, score_vec, level_vec, angle_vec, type_vec,
        descriptors, num_detected);
  FastDetectorOptions fast_options;
  fast_options.threshold = 20.0f;
  FastDetectorAF detector(fast_options, af_im->size());

  detector.detect(*pyr, features); // GPU warm-up
  auto detectLambda = [&](){
    features.num_detected = 0u; // Reset.
    detector.detect(*pyr, features);
  };
  runTimingBenchmark(detectLambda, 10, 20, "AF FAST Detector", true);

#define ZE_TEST_FAST_AF_SHOW 0

#if ZE_TEST_FAST_AF_SHOW
  const int draw_len = 3;
#endif
  for (int8_t l=0; l<static_cast<int8_t>(pyr->numLevels()); ++l)
  {
    const auto& lvl = dynamic_cast<ImageAF8uC1&>(pyr->at(l));
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

TEST(impBridgeAFTest, siftDetectorAF32fC1)
{
  std::string path(
        joinPath(
          getTestDataDir(g_test_data_name),
          g_predefined_img_data_file_name));

  ImageCv32fC1::Ptr cv_img;
  cvBridgeLoad(
        cv_img,
        path,
        PixelOrder::gray);

  ImageAF32fC1::Ptr im =
      std::make_shared<ImageAF32fC1>(*cv_img);

  SiftDetectorOptions options;
  SiftDetectorAF detector(options, im->size());
  SiftKeypointWrapper::Ptr features;
  detector.detect(*im, features); // GPU warm-up
  auto detectLambda = [&](){
    detector.detect(*im, features);
  };
  runTimingBenchmark(detectLambda, 10, 20, "AF SIFT Detector", true);
}

TEST(impBridgeAFTest, orbDetectorAF32fC1)
{
  std::string path(
        joinPath(
          getTestDataDir(g_test_data_name),
          g_predefined_img_data_file_name));

  ImageCv32fC1::Ptr cv_img;
  cvBridgeLoad(
        cv_img,
        path,
        PixelOrder::gray);

  ImageAF32fC1::Ptr im =
      std::make_shared<ImageAF32fC1>(*cv_img);

  OrbDetectorOptions options;
  options.fast_thr /= 255.f;
  OrbDetectorAF detector(options, im->size());
  OrbKeypointWrapper::Ptr features;
  detector.detect(*im, features); // GPU warm-up
  auto detectLambda = [&](){
    detector.detect(*im, features);
  };
  runTimingBenchmark(detectLambda, 10, 20, "AF ORB Detector", true);
}

ZE_UNITTEST_ENTRYPOINT
