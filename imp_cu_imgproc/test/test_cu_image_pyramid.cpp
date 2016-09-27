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

#include <imp/core/image.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>
#include <imp/cu_imgproc/image_pyramid.hpp>

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
