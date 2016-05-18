#include <arrayfire.h>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/af/feature_detection.hpp>
#include <ze/common/test_entrypoint.h>
#include <ze/common/benchmark.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_utils.h>

namespace ze {
namespace test {

constexpr const char* g_test_data_name =
    "ze_feature_detection";

template<typename Pixel>
typename ze::ImageCv<Pixel>::Ptr loadTestImg(
    ze::PixelOrder pixel_order,
    const std::string& path)
{
  CHECK(ze::fileExists(path))
      << ", path: '" << path << "'";
  typename ze::ImageCv<Pixel>::Ptr cv_img;
  ze::cvBridgeLoad(cv_img, path, pixel_order);
  VLOG(2) << "loaded image " << path
          << ", size " << cv_img->size();
  return cv_img;
}

} // test namespace
} // ze namespace

TEST(impAfBridge, createArrayFromImp)
{
  ze::ImageCv32fC1::Ptr cv_img =
      ze::test::loadTestImg<ze::Pixel32fC1>(
        ze::PixelOrder::gray,
        ze::joinPath(
          ze::getTestDataDir(ze::test::g_test_data_name),
          "752x480/pyr_-1.png"));
  ze::cu::ImageGpu32fC1::Ptr in_img =
      std::make_shared<ze::cu::ImageGpu32fC1>(*cv_img);
  af::array a = ze::cu::createFromImp(*in_img);
}

TEST(impAfBridge, afSum)
{
  ze::ImageCv32fC1::Ptr cv_img =
      ze::test::loadTestImg<ze::Pixel32fC1>(
        ze::PixelOrder::gray,
        ze::joinPath(
          ze::getTestDataDir(ze::test::g_test_data_name),
          "752x480/pyr_0.png"));
  ze::cu::ImageGpu32fC1::Ptr in_img =
      std::make_shared<ze::cu::ImageGpu32fC1>(*cv_img);
  af::array a = ze::cu::createFromImp(*in_img);
  // Sum the values and copy the result to the CPU:
  double sum = af::sum<float>(a);
  // Compute ground truth and compare
  double gt_sum = 0.0;
  for (size_t r=0; r<cv_img->height(); ++r)
  {
    for (size_t c=0; c<cv_img->width(); ++c)
    {
      gt_sum += cv_img->pixel(c, r);
    }
  }
  EXPECT_EQ(gt_sum, sum);
  VLOG(1) << "sum: " << sum;
  VLOG(1) << "GT sum: " << gt_sum;
}

ZE_UNITTEST_ENTRYPOINT
