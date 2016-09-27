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
#include <imp/cu_core/cu_math.cuh>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>
#include <imp/cu_imgproc/str_tex_decomposer.hpp>

DEFINE_bool(visualize, false, "Show input images and results");

template <typename Pixel>
class CuStrTexDenoiseTestFixture : public ::testing::Test
{
protected:
  CuStrTexDenoiseTestFixture()
    : data_path_(ze::getTestDataDir("computer_vision_images"))
  {
  }

  void loadLenaGrayscale()
  {
    ze::cu::cvBridgeLoad(
          in_, data_path_ + "/lena_grayscale_511x512.png", ze::PixelOrder::gray);
  }

  void strTexDecompose()
  {
    tex_ = std::make_shared<ze::cu::ImageGpu<Pixel>>(in_->size());
    str_ = std::make_shared<ze::cu::ImageGpu<Pixel>>(in_->size());
    str_tex_decomp_ = std::make_shared<ze::cu::StrTexDecomposer<Pixel>>();
    str_tex_decomp_->solve(tex_, in_, str_);

    if (VLOG_IS_ON(2))
    {
      Pixel min, max;
      ze::cu::minMax(*tex_, min, max);
      VLOG(2) << "tex_ val range: [" << min << ", " << max << "]";
      ze::cu::minMax(*str_, min, max);
      VLOG(2) << "str_ val range: [" << min << ", " << max << "]";
    }
  }

protected:
  std::string data_path_;
  ze::cu::StrTexDecomposerPtr<Pixel> str_tex_decomp_;
  ze::cu::ImageGpuPtr<Pixel> str_, tex_, in_;
};


// The list of types we want to test.
typedef testing::Types<
ze::Pixel8uC1, ze::Pixel32fC1
> PixelTypes;

TYPED_TEST_CASE(CuStrTexDenoiseTestFixture, PixelTypes);

TYPED_TEST(CuStrTexDenoiseTestFixture, testVisStrTexDecomp)
{
  using namespace ze::cu;

  this->loadLenaGrayscale();
  this->strTexDecompose();

  if(FLAGS_visualize)
  {
    cvBridgeShow("Lena", *this->in_);
    std::stringstream windowname;
    windowname << "Texture part of Lena ("
               << static_cast<int>(this->in_->bitDepth()) << "-bit)";
    cvBridgeShow(windowname.str(), *this->tex_);
    windowname.str(std::string());
    windowname << "Structure part of Lena ("
               << static_cast<int>(this->in_->bitDepth()) << "-bit)";
    cvBridgeShow(windowname.str(), *this->str_);
    cv::waitKey();
  }
}

ZE_UNITTEST_ENTRYPOINT
