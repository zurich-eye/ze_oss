#include <assert.h>
#include <cstdint>
#include <iostream>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <imp/core/roi.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_math.cuh>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>

int main(int argc, char** argv)
{
  try
  {
    if (argc < 2)
    {
      std::cout << "usage: cu_rof_denoising_test input_image_filename";
      return EXIT_FAILURE;
    }
    std::string in_filename(argv[1]);


    // get image minmax
    {
      std::shared_ptr<imp::cu::ImageGpu8uC1> cu_im;
      imp::cu::cvBridgeLoad(cu_im, in_filename, imp::PixelOrder::gray);
      imp::Pixel8uC1 min_pixel, max_pixel;
      imp::cu::minMax(*cu_im, min_pixel, max_pixel);
      std::cout << "min: " << (int)min_pixel << ", max: " << (int)max_pixel << std::endl;
    }

  }
  catch (std::exception& e)
  {
    std::cout << "[exception] " << e.what() << std::endl;
    cudaDeviceReset();
    assert(false);
  }
  cudaDeviceReset();
  return EXIT_SUCCESS;
}
