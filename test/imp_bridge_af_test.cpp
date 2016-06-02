#include <imp/bridge/af/orb_detector_af.hpp>

#include <imp/bridge/opencv/cv_bridge.hpp>

#include <ze/common/benchmark.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

using namespace ze;

TEST(impBridgeAFTest, constructFromImpImage_32fC1)
{
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
  ImageAF32fC1 af_img(*cv_img);
  double af_sum = af::sum<double>(af_img.afArray());
  VLOG(1) << std::fixed << "AF sum: " << af_sum;
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  VLOG(1) << std::fixed << "OpenCV sum: " << cv_sum;
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

TEST(impBridgeAFTest, constructFromImpImage_8uC1)
{
  const std::string test_data_name{"ze_feature_detection"};
  const std::string predefined_img_data_file_name{"752x480/pyr_0.png"};

  std::string path(
        joinPath(
          getTestDataDir(test_data_name),
          predefined_img_data_file_name));

  ImageCv8uC1::Ptr cv_img;
  cvBridgeLoad(cv_img, path, PixelOrder::gray);
  VLOG(2) << "loaded image " << path
          << ", size " << cv_img->size();
  ImageAF8uC1 af_img(*cv_img);
  double af_sum = af::sum<double>(af_img.afArray());
  VLOG(1) << std::fixed << "AF sum: " << af_sum;
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  VLOG(1) << std::fixed << "OpenCV sum: " << cv_sum;
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

TEST(impBridgeAFTest, constructFromAFArray_32fC1)
{
  const std::string test_data_name{"ze_feature_detection"};
  const std::string predefined_img_data_file_name{"752x480/pyr_0.png"};

  std::string path(
        joinPath(
          getTestDataDir(test_data_name),
          predefined_img_data_file_name));

  ImageCv32fC1::Ptr cv_img;
  cvBridgeLoad(cv_img, path, PixelOrder::gray);

  std::unique_ptr<float[]> h_buffer(
        new float[cv_img->width()*cv_img->height()]);
  for (size_t y = 0; y < cv_img->height(); ++y)
  {
    for (size_t x = 0; x < cv_img->width(); ++x)
    {
      h_buffer.get()[x*cv_img->height()+y] = cv_img->pixel(x, y);
    }
  }
  ImageAF32fC1 af_img(
        af::array(
          cv_img->height(),
          cv_img->width(),
          h_buffer.get()));
  double af_sum = af::sum<double>(af_img.afArray());
  VLOG(1) << std::fixed << "AF sum: " << af_sum;
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  VLOG(1) << std::fixed << "OpenCV sum: " << cv_sum;
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

TEST(impBridgeAFTest, constructFromAFArray_8uC1)
{
  const std::string test_data_name{"ze_feature_detection"};
  const std::string predefined_img_data_file_name{"752x480/pyr_0.png"};

  std::string path(
        joinPath(
          getTestDataDir(test_data_name),
          predefined_img_data_file_name));

  ImageCv8uC1::Ptr cv_img;
  cvBridgeLoad(cv_img, path, PixelOrder::gray);

  std::unique_ptr<unsigned char[]> h_buffer(
        new unsigned char[cv_img->width()*cv_img->height()]);
  for (size_t y = 0; y < cv_img->height(); ++y)
  {
    for (size_t x = 0; x < cv_img->width(); ++x)
    {
      h_buffer.get()[x*cv_img->height()+y] = cv_img->pixel(x, y);
    }
  }
  ImageAF8uC1 af_img(
        af::array(
          cv_img->height(),
          cv_img->width(),
          h_buffer.get()));
  double af_sum = af::sum<double>(af_img.afArray());
  VLOG(1) << std::fixed << "AF sum: " << af_sum;
  double cv_sum = cv::sum(cv_img->cvMat())[0];
  VLOG(1) << std::fixed << "OpenCV sum: " << cv_sum;
  EXPECT_NEAR(cv_sum, af_sum, 0.01);
}

TEST(impBridgeAFTest, orbDetectorAF32fC1)
{
  const std::string test_data_name{"ze_feature_detection"};
  const std::string predefined_img_data_file_name{"752x480/pyr_0.png"};

  std::string path(joinPath(getTestDataDir(test_data_name),
                            predefined_img_data_file_name));

  ImageCv32fC1::Ptr cv_img;
  cvBridgeLoad(cv_img, path, PixelOrder::gray);

  ImageAF32fC1::Ptr im =
      std::make_shared<ImageAF32fC1>(*cv_img);

  OrbDetectorOptions options;
  options.fast_threshold /= 255.f;
  OrbDetectorAF detector(options, im->size());
  OrbKeypointWrapper features;
  detector.detect(*im, features); // GPU warm-up
  auto detectLambda = [&](){
    detector.detect(*im, features);
  };
  runTimingBenchmark(detectLambda, 10, 20, "AF ORB Detector", true);

  Keypoints keypoints = features.getKeypoints();
  KeypointScores scores = features.getKeypointScores();
  KeypointSizes sizes = features.getKeypointSizes();
  KeypointAngles angles = features.getKeypointAngles();
  OrbDescriptors descriptors = features.getDescriptors();

  for (int k = 0; k < keypoints.cols(); ++k)
  {
    EXPECT_GT(keypoints(0, k), 0);
    EXPECT_LT(keypoints(0, k), im->width());
    EXPECT_GT(keypoints(1, k), 0);
    EXPECT_LT(keypoints(1, k), im->height());
  }
  VLOG(2) << "number of computed descriptors: " << descriptors.cols();
}

ZE_UNITTEST_ENTRYPOINT
