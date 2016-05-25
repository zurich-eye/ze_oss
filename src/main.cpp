#include <arrayfire.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/af/fast_detector_af.hpp>
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

  ze::ImageAF8uC1::Ptr im =
      std::make_shared<ze::ImageAF8uC1>(*cv_img);
  ze::ImagePyramid8uC1::Ptr pyr =
      ze::createAFImagePyramid<ze::Pixel8uC1>(im, 0.5, 5, 8);

  uint32_t max_fts = 300u;
  ze::Keypoints px_vec(2, max_fts);
  ze::KeypointScores score_vec(max_fts);
  ze::KeypointLevels level_vec(max_fts);
  ze::KeypointAngles angle_vec(max_fts);
  ze::KeypointTypes type_vec(max_fts);
  ze::Descriptors descriptors;
  uint32_t num_detected = 0u;
  ze::FastDetectorOptions fast_options;
  ze::KeypointsWrapper features(
        px_vec, score_vec, level_vec, angle_vec, type_vec,
        descriptors, num_detected);

  ze::FastDetectorAF detector(fast_options, im->size());
  detector.detect(*pyr, features);

  for (int8_t l=0; l<static_cast<int8_t>(pyr->numLevels()); ++l)
  {
    for (size_t f=0; f<features.num_detected; ++f)
    {
      //if (l == features.levels(f))
      {
        printf("Level %i, x=%f, y=%f, score=%f\n", l, features.px(0, f), features.px(1, f), features.scores(f));
      }
    }

    const auto& lvl = dynamic_cast<ze::ImageAF8uC1&>(pyr->at(l));
    af::Window wnd("AF array");
    while(!wnd.close())
      wnd.image(lvl.afArray());
  }
  return 0;
}
