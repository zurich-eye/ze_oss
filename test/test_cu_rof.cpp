#include <sstream>
#include <string>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/benchmark.hpp>
#include <ze/common/types.hpp>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>
#include <imp/cu_imgproc/cu_rof_denoising.cuh>

DEFINE_bool(visualize, false, "Show input images and results");

template <typename Pixel>
class CuRofDenoiseTestFixture : public ::testing::Test
{
protected:
  CuRofDenoiseTestFixture()
    : data_path_(ze::getTestDataDir("computer_vision_images"))
  {
  }

  void loadLenaGrayscale()
  {
    ze::cu::cvBridgeLoad(
          in_, data_path_ + "/lena_grayscale_511x512.png", ze::PixelOrder::gray);
  }

  void denoise()
  {
    denoised_ = std::make_shared<ze::cu::ImageGpu<Pixel>>(in_->size());
    rof_ = std::make_shared<ze::cu::RofDenoising<Pixel>>();

    rof_->params().primal_dual_energy_check_iter = 10;
    rof_->params().primal_dual_gap_tolerance = 1e-3;

    rof_->denoise(denoised_, in_);
  }

protected:
  std::string data_path_;
  ze::cu::RofDenoisingPtr<Pixel> rof_;
  ze::cu::ImageGpuPtr<Pixel> in_;
  ze::cu::ImageGpuPtr<Pixel> denoised_;
};


// The list of types we want to test.
typedef testing::Types<
ze::Pixel8uC1, ze::Pixel32fC1
> PixelTypes;

TYPED_TEST_CASE(CuRofDenoiseTestFixture, PixelTypes);

TYPED_TEST(CuRofDenoiseTestFixture, VisualTestRofDenoising)
{
  using namespace ze::cu;

  this->loadLenaGrayscale();
  this->denoise();

  if(FLAGS_visualize)
  {
    cvBridgeShow("Lena", *this->in_);
    std::stringstream windowname;
    windowname << "ROF Lena ("
               << static_cast<int>(this->denoised_->bitDepth()) << "-bit)";
    cvBridgeShow(windowname.str(), *this->denoised_);
    cv::waitKey();
  }
}

ZE_UNITTEST_ENTRYPOINT
