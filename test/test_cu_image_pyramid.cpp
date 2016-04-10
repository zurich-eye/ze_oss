#include <sstream>
#include <string>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/common/benchmark.h>
#include <ze/common/types.h>

#include <imp/core/image.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>
#include <imp/cu_imgproc/image_pyramid.cuh>

DEFINE_bool(visualize, false, "Show input images and results");

TEST(ImagePyramidTest, testImagePyramidGPU)
{
  using namespace ze::cu;

  std::string data_path = ze::getTestDataDir("computer_vision_images");
  ImageGpu32fC1::Ptr cuimg;
  cvBridgeLoad(cuimg, data_path + "/lena_grayscale_511x512.png", ze::PixelOrder::gray);

  auto pyr = createImagePyramidGpu<ze::Pixel32fC1>(cuimg, 0.8);

  VLOG(200) << "IMAGE PYRAMID: num levels: " << pyr->numLevels();

  if(FLAGS_visualize)
  {
    for (size_t level=0; level < pyr->numLevels(); ++level)
    {
      std::stringstream windowname;
      VLOG(200) << "showing pyramid level " << level;
      windowname << "praymid level " << level;
      cvBridgeShow(windowname.str(), dynamic_cast<ImageGpu32fC1&>(pyr->at(level)));
    }
    cv::waitKey(0);
  }
}

ZE_UNITTEST_ENTRYPOINT
