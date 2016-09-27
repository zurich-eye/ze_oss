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
#include <imp/cu_imgproc/cu_rof_denoising.cuh>
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


    // ROF denoising 8uC1
    {
      std::shared_ptr<ze::cu::ImageGpu8uC1> cu_im;
      ze::cu::cvBridgeLoad(cu_im, in_filename, ze::PixelOrder::gray);
      std::shared_ptr<ze::cu::ImageGpu8uC1> cu_im_denoised(
            new ze::cu::ImageGpu8uC1(*cu_im));

      ze::cu::RofDenoising8uC1 rof;
      rof.params().primal_dual_energy_check_iter = 10;
      rof.params().primal_dual_gap_tolerance = 1e-3;

      std::cout << "\n" << rof << std::endl;
      rof.denoise(cu_im_denoised, cu_im);

      // show results
      ze::cu::cvBridgeShow("input 8u", *cu_im);
      ze::cu::cvBridgeShow("denoised 8u", *cu_im_denoised);
    }

    std::cout << "-------------------------------------------------------------"
              << std::endl << std::endl;

    // ROF denoising 32fC1
    {
      std::shared_ptr<ze::cu::ImageGpu32fC1> cu_im;
      ze::cu::cvBridgeLoad(cu_im, in_filename, ze::PixelOrder::gray);
      std::shared_ptr<ze::cu::ImageGpu32fC1> cu_im_denoised(
            new ze::cu::ImageGpu32fC1(*cu_im));

      ze::cu::RofDenoising32fC1 rof;
      rof.params().primal_dual_energy_check_iter = 10;
      rof.params().primal_dual_gap_tolerance = 1e-3;

      std::cout << "\n" << rof << std::endl;
      rof.denoise(cu_im_denoised, cu_im);

      ze::cu::cvBridgeShow("input 32f", *cu_im);
      ze::cu::cvBridgeShow("denoised 32f", *cu_im_denoised);
    }

    cv::waitKey();
  }
  catch (std::exception& e)
  {
    std::cout << "[exception] " << e.what() << std::endl;
    assert(false);
  }

  return EXIT_SUCCESS;

}
