#include <arrayfire.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/af/fast_detector_af.hpp>
#include <imp/bridge/af/image_af.hpp>
#include <imp/bridge/af/pyramid_af.hpp>
#include <imp/bridge/af/sift_detector_af.hpp>

constexpr const char* img_file_path = "/home/mpi/workspace/arrayfire/assets/examples/images/man.jpg";

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ze::ImageCv32fC1::Ptr cv_img;
  ze::cvBridgeLoad(
        cv_img,
        img_file_path,
        ze::PixelOrder::gray);

  ze::ImageAF32fC1::Ptr im =
      std::make_shared<ze::ImageAF32fC1>(*cv_img);

  ze::SiftDetectorOptions options;
  ze::SiftDetectorAF detector(options, im->size());
  ze::SiftKeypointWrapper::Ptr features;
  detector.detect(*im, features);
  ze::SiftKeypointWrapper::Descriptors descr = features->getDescriptors();
  return 0;
}
