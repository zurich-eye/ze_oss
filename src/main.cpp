#include <arrayfire.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/af/feature_detection.hpp>

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ze::ImageCv32fC1::Ptr cv_img;
  ze::cvBridgeLoad(
        cv_img,
        "/home/mpi/workspace/zurich_eye_ws/src/ze_test_data/data/ze_feature_detection/752x480/pyr_4.png",
        ze::PixelOrder::gray);

  ze::cu::ImageGpu32fC1::Ptr in_img =
      std::make_shared<ze::cu::ImageGpu32fC1>(*cv_img);

  VLOG(1) << "loaded image size: " << in_img->size();

  af::array a = ze::cu::createFromImp(*in_img);

  // Sum the values and copy the result to the CPU:
  double sum = af::sum<float>(a);

  double gt_sum = 0.0;
  for (size_t r=0; r<cv_img->height(); ++r)
  {
    for (size_t c=0; c<cv_img->width(); ++c)
    {
      gt_sum += cv_img->pixel(c, r);
    }
  }

  VLOG(1) << "sum: " << sum;
  VLOG(1) << "GT sum: " << gt_sum;

  //! TODO (MPI): who takes care of freeing gpu memory?
  return 0;
}
