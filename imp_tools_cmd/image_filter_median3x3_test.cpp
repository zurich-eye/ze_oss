// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
    ze::ImageCv8uC1 h1_lena_8uC1(cv::imread("/home/mwerlberger/data/std/Lena.tiff",
                                             CV_LOAD_IMAGE_GRAYSCALE),
                                  ze::PixelOrder::gray);

    // add salt and pepper noise
    addImpulseNoise(h1_lena_8uC1.cvMat(), 20);

    {
      // copy host->device
      std::unique_ptr<ze::cu::ImageGpu8uC1> d_lena_8uC1(
            new ze::cu::ImageGpu8uC1(h1_lena_8uC1));

      std::unique_ptr<ze::cu::ImageGpu8uC1> d_median_lena_8uC1(
            new ze::cu::ImageGpu8uC1(d_lena_8uC1->size()));

      ze::cu::filterMedian3x3(*d_median_lena_8uC1, *d_lena_8uC1);

      ze::cu::cvBridgeShow("lena 8u", *d_lena_8uC1);
      ze::cu::cvBridgeShow("lena median 8u", *d_median_lena_8uC1);

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
