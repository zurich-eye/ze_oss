#include <arrayfire.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/af/feature_detection.hpp>

#include <imp/bridge/af/image_af.hpp>

constexpr const char* img_file_path = "/home/mpi/workspace/zurich_eye_ws/src/ze_test_data/data/ze_feature_detection/752x480/pyr_0.png";

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

#if 0
  ze::ImageCv32fC1::Ptr cv_img;
  ze::cvBridgeLoad(
        cv_img,
        img_file_path,
        ze::PixelOrder::gray);

  ze::cu::ImageGpu32fC1::Ptr in_img =
      std::make_shared<ze::cu::ImageGpu32fC1>(*cv_img);

  VLOG(1) << "loaded IMP image size: " << in_img->size();

  af::array b = af::loadImage(img_file_path);
  b /= 255.0;

  VLOG(1) << "loaded AF image dims: " << b.dims();

  af::array a = ze::cu::createFromImp(*in_img);

  // Sum the values and copy the result to the CPU:
  double a_sum = af::sum<float>(a);
  double b_sum = af::sum<float>(b);
  cv::Scalar cv_sum = cv::sum(cv_img->cvMat());

  double gt_sum = 0.0;
  for (size_t r=0; r<cv_img->height(); ++r)
  {
    for (size_t c=0; c<cv_img->width(); ++c)
    {
      gt_sum += cv_img->pixel(c, r);
    }
  }

  printf("a sum: %f\n", a_sum);
  printf("b sum: %f\n", b_sum);
  printf("CV sum: %f\n", cv_sum(0));
  printf("GT sum: %f\n", gt_sum);

  //! TODO (MPI): who takes care of freeing gpu memory?
#endif

  ze::ImageAF32fC1 im(752, 480);
  im.fast();

  return 0;
}
