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

  std::shared_ptr<imp::ImageRaw8uC1> im_8uC1;
  imp::pangolinBridgeLoad(im_8uC1, filename, imp::PixelOrder::gray);

  VLOG(2) << "Read Lena (png) from " << filename
          << ": " << im_8uC1->width() << "x" << im_8uC1->height() << "(" << im_8uC1->pitch() << ")";

  imp::imshow(*im_8uC1, "Lena");

  return EXIT_SUCCESS;
}

