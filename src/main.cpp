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

  uint32_t max_fts = 6000u;
  ze::Keypoints px_vec(2, max_fts);
  ze::KeypointScores score_vec(max_fts);
  ze::KeypointLevels level_vec(max_fts);
  ze::KeypointAngles angle_vec(max_fts);
  ze::KeypointTypes type_vec(max_fts);
  ze::Descriptors descriptors;
  uint32_t num_detected = 0u;
  ze::KeypointsWrapper features(
        px_vec, score_vec, level_vec, angle_vec, type_vec,
        descriptors, num_detected);
  ze::FastDetectorOptions fast_options;
  fast_options.threshold = 20.0f;
  ze::FastDetectorAF detector(fast_options, im->size());
  detector.detect(*pyr, features);

  const int draw_len = 3;
  for (int8_t l=0; l<static_cast<int8_t>(pyr->numLevels()); ++l)
  {
    const auto& lvl = dynamic_cast<ze::ImageAF8uC1&>(pyr->at(l));
    af::array display_arr = af::colorSpace(lvl.afArray(), AF_RGB, AF_GRAY)/255.f;

    for (size_t f=0; f<features.num_detected; ++f)
    {
      if (l == features.levels(f))
      {
        const int x = features.px(0, f) * pyr->scaleFactor(l);
        const int y = features.px(1, f) * pyr->scaleFactor(l);
        printf("Level %i, x=%i, y=%i, score=%f\n", features.levels(f), x, y, features.scores(f));
        display_arr(y, af::seq(x-draw_len, x+draw_len), 0) = 0.f;
        display_arr(y, af::seq(x-draw_len, x+draw_len), 1) = 1.f;
        display_arr(y, af::seq(x-draw_len, x+draw_len), 2) = 0.f;

        // Draw vertical line of (draw_len * 2 + 1) pixels centered on  the corner
        // Set only the first channel to 1 (green lines)
        display_arr(af::seq(y-draw_len, y+draw_len), x, 0) = 0.f;
        display_arr(af::seq(y-draw_len, y+draw_len), x, 1) = 1.f;
        display_arr(af::seq(y-draw_len, y+draw_len), x, 2) = 0.f;
      }
    }

    af::Window wnd("AF array");
    while(!wnd.close())
      wnd.image(display_arr);
  }
  return 0;
}
