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
