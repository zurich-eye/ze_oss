#include <arrayfire.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/af/feature_detection.hpp>
#include <imp/bridge/af/image_af.hpp>
#include <imp/bridge/af/pyramid_af.hpp>

constexpr const char* img_file_path = "/home/mpi/workspace/arrayfire/assets/examples/images/man.jpg";

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ze::ImageCv8uC1::Ptr cv_img;
  ze::cvBridgeLoad(
        cv_img,
        img_file_path,
        ze::PixelOrder::gray);

  ze::ImageAF8uC1::Ptr im = std::make_shared<ze::ImageAF8uC1>(*cv_img);
  ze::ImagePyramid8uC1::Ptr pyr = ze::createAFImagePyramid<ze::Pixel8uC1>(im, 0.5, 5, 8);

  for (size_t l=0; l<pyr->numLevels(); ++l)
  {
    const auto& lvl = dynamic_cast<ze::ImageAF8uC1&>(pyr->at(l));
    af::Window wnd("AF array");
    while(!wnd.close())
      wnd.image(lvl.afArray());
  }
  return 0;
}
