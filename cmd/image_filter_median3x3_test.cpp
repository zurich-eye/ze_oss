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
#include <imp/cu_imgproc/cu_image_filter.cuh>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>


void addImpulseNoise(cv::Mat& img, double perc)
{
  const int rows = img.rows;
  const int cols = img.cols;
  const int channels = img.channels();
  int num_corrupted_pts = static_cast<int>((rows*cols*channels)*perc/100.0);

  for (int i=0; i<num_corrupted_pts; ++i)
  {
    int r = rand() % rows;
    int c = rand() % cols;
    int channel = rand() % channels;

    uchar* pixel = img.ptr<uchar>(r) + (c*channels) + channel;
    *pixel = (rand()%2) ? 255 : 0;
  }
}

int main(int /*argc*/, char** /*argv*/)
{
  try
  {
    imp::ImageCv8uC1 h1_lena_8uC1(cv::imread("/home/mwerlberger/data/std/Lena.tiff",
                                             CV_LOAD_IMAGE_GRAYSCALE),
                                  imp::PixelOrder::gray);

    // add salt and pepper noise
    addImpulseNoise(h1_lena_8uC1.cvMat(), 20);

    {
      // copy host->device
      std::unique_ptr<imp::cu::ImageGpu8uC1> d_lena_8uC1(
            new imp::cu::ImageGpu8uC1(h1_lena_8uC1));

      std::unique_ptr<imp::cu::ImageGpu8uC1> d_median_lena_8uC1(
            new imp::cu::ImageGpu8uC1(d_lena_8uC1->size()));

      imp::cu::filterMedian3x3(*d_median_lena_8uC1, *d_lena_8uC1);

      imp::cu::cvBridgeShow("lena 8u", *d_lena_8uC1);
      imp::cu::cvBridgeShow("lena median 8u", *d_median_lena_8uC1);

      cv::waitKey();
    }
  }
  catch (std::exception& e)
  {
    std::cout << "[exception] " << e.what() << std::endl;
    assert(false);
  }

  return EXIT_SUCCESS;

}
