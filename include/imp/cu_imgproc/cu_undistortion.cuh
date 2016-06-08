#pragma once
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <ze/cameras/camera_models.h>

namespace ze {
namespace cu {

class ImageUndistorter
{
public:
  ImageUndistorter(/*int32_t width, int32_t height*/)
  {
  }
  ~ImageUndistorter()
  {
  }

  void undistort(const ImageCv32fC1& in/*, Image32fC1& out*/)
  {
    cv::Mat und = in.cvMat().clone();

    for (int y = 0; y < und.rows; ++y)
    {
      for (int x = 0; x < und.cols; ++x)
      {
        float px[2];
        px[0] = x;
        px[1] = y;
        PinholeGeometry::backProject(params_, px);
        EquidistantDistortion::distort(dists_, px);
        PinholeGeometry::project(params_, px);

        if (px[0]>=0 && px[0]<und.cols && px[1]>=0 && px[1]<und.rows)
        {
          und.at<float>(y, x) = in.cvMat().at<float>(px[1], px[0]);
        }
      }
    }
    cv::imshow("orig", in.cvMat());
    cv::imshow("test", und);
    cv::waitKey(0);
  }


private:
  // ze::cu::Fragmentation<> fragm_;
  float params_[4] = {471.690643292, 471.765601046, 371.087464172, 228.63874151};
  float dists_[4] = {0.00676530475436, -0.000811126898338, 0.0166458761987, -0.0172655346139};
};

} // cu namespace
} // ze namespace
