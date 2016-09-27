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

#include <glog/logging.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <pangolin/image.h>
//#include <pangolin/image_load.h>
//#define BUILD_PANGOLIN_GUI
//#include <pangolin/config.h>
#include <pangolin/pangolin.h>

#include <imp/core/roi.hpp>
#include <imp/core/image_raw.hpp>
//#include <imp/bridge/opencv/image_cv.hpp>
//#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/bridge/pangolin/imread.hpp>
#include <imp/bridge/pangolin/pangolin_display.hpp>



void setImageData(unsigned char * imageArray, int size){
  for(int i = 0 ; i < size;i++) {
    imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
  }
}

int main( int argc, char* argv[] )
{
  google::InitGoogleLogging(argv[0]);
  CHECK_GE(argc,2) << "Usage: pangolin_load_test image_location";
  const std::string filename(argv[1]);

  std::shared_ptr<ze::ImageRaw8uC1> im_8uC1;
  ze::pangolinBridgeLoad(im_8uC1, filename, ze::PixelOrder::gray);

  VLOG(2) << "Read Lena (png) from " << filename
          << ": " << im_8uC1->width() << "x" << im_8uC1->height() << "(" << im_8uC1->pitch() << ")";

  ze::imshow(*im_8uC1, "Lena");

  return EXIT_SUCCESS;
}

