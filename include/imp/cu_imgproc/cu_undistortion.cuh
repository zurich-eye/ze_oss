#pragma once
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <ze/cameras/camera_models.h>

namespace ze {
namespace cu {

template <typename CameraModel,
          typename DistortionModel,
          typename T>
class ImageUndistorter
{
public:
  ImageUndistorter(int32_t width, int32_t height, T* camera_params, T* dist_coeffs)
  {
    x_map_.create(height, width);
    y_map_.create(height, width);
    for (int y = 0; y < x_map_.rows; ++y)
    {
      for (int x = 0; x < x_map_.cols; ++x)
      {
        float px[2];
        px[0] = x;
        px[1] = y;
        CameraModel::backProject(camera_params, px);
        DistortionModel::distort(dist_coeffs, px);
        CameraModel::project(camera_params, px);
        x_map_(y, x) = px[0];
        y_map_(y, x) = px[1];
      }
    }
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
        px[0] = x_map_(y, x);
        px[1] = y_map_(y, x);
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
  cv::Mat_<float> x_map_;
  cv::Mat_<float> y_map_;
  // ze::cu::Fragmentation<> fragm_;
};

} // cu namespace
} // ze namespace
