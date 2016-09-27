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
//#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_imgproc/cu_image_filter.cuh>
//#include <imp/bridge/opencv/cu_cv_bridge.hpp>

int main(int argc, char** argv)
{
  try
  {
    if (argc < 2)
    {
      std::cout << "usage: " << argv[0] << " input_image_filename";
      return EXIT_FAILURE;
    }
    std::string in_filename(argv[1]);

    ze::ImageCv8uC1 h1_lena_8uC1(cv::imread(in_filename, CV_LOAD_IMAGE_GRAYSCALE), ze::PixelOrder::gray);

    {
      // copy host->device
      std::unique_ptr<ze::cu::ImageGpu8uC1> d_lena_8uC1(
            new ze::cu::ImageGpu8uC1(h1_lena_8uC1));

      std::unique_ptr<ze::cu::ImageGpu8uC1> d_gauss_lena_8uC1(
            new ze::cu::ImageGpu8uC1(d_lena_8uC1->size()));

      ze::cu::filterGauss(*d_gauss_lena_8uC1, *d_lena_8uC1, 10.0);


      ze::ImageCv8uC1 h_gauss_lena_8uC1(*d_gauss_lena_8uC1);
      cv::imshow("lena 8u", h1_lena_8uC1.cvMat());
      cv::imshow("lena gauss 8u", h_gauss_lena_8uC1.cvMat());

    }

    {
      // 32fC1 test
      ze::ImageCv32fC1 h1_lena_32fC1(h1_lena_8uC1.size());

      h1_lena_8uC1.cvMat().convertTo(h1_lena_32fC1.cvMat(), CV_32F);
      h1_lena_32fC1.cvMat() /= 255.f;

      // copy host->device
      std::unique_ptr<ze::cu::ImageGpu32fC1> d_lena_32fC1(
            new ze::cu::ImageGpu32fC1(h1_lena_32fC1));

      std::unique_ptr<ze::cu::ImageGpu32fC1> d_gauss_lena_32fC1(
            new ze::cu::ImageGpu32fC1(d_lena_32fC1->size()));

      ze::cu::filterGauss(*d_gauss_lena_32fC1, *d_lena_32fC1, 10.0);


      ze::ImageCv32fC1 h_gauss_lena_32fC1(*d_gauss_lena_32fC1);
      cv::imshow("lena 32f", h1_lena_32fC1.cvMat());
      cv::imshow("lena gauss 32f", h_gauss_lena_32fC1.cvMat());

    }

//    // 32fC1 texture
//    {
//      std::shared_ptr<ze::cu::ImageGpu32fC1> d_lena;
//      ze::cu::cvBridgeLoad(d_lena, "/home/mwerlberger/data/std/Lena.tiff",
//                            ze::PixelOrder::gray);
//      std::shared_ptr<ze::cu::ImageGpu32fC1> d_lena_denoised(
//            new ze::cu::ImageGpu32fC1(d_lena->size()));

//      std::shared_ptr<ze::cu::Texture2D> d_tex_lena =
//          d_lena->genTexture(false, cudaFilterModeLinear);

//      ze::cu::filterGauss(*d_lena_denoised, *d_tex_lena, 10.0);

//      ze::cu::cvBridgeShow("lena input 32f texture", *d_lena);
//      ze::cu::cvBridgeShow("lena denoised 32f texture", *d_lena_denoised);
//    }
    cv::waitKey();
  }
  catch (std::exception& e)
  {
    std::cout << "[exception] " << e.what() << std::endl;
    assert(false);
  }

  return EXIT_SUCCESS;

}
