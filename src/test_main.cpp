#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <ze/cameras/camera.h>
#include <ze/common/file_utils.h>
#include <ze/common/test_utils.h>

#include <imp/cu_imgproc/cu_undistortion.cuh>

int main(int argc, char** argv)
{
  using namespace ze;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Load image to gpu
  const std::string test_data_name{"ze_feature_detection"};
  const std::string predefined_img_data_file_name{"752x480/pyr_0.png"};

  std::string path(
        joinPath(
          getTestDataDir(test_data_name),
          predefined_img_data_file_name));
  ImageCv32fC1::Ptr cv_img;
  cvBridgeLoad(cv_img, path, PixelOrder::gray);
  VLOG(2) << "loaded image " << path
          << ", size " << cv_img->size();
//  ze::cu::ImageGpu32fC1 cu_im(*cv_img);

  // Load camera parameters
  std::string yaml_file_path = joinPath(test_data_name, "752x480/visensor_22030_swe_params.yaml");
//  Camera::Ptr cam = Camera::loadFromYaml(yaml_file_path);

  float params[4] = {471.690643292, 471.765601046, 371.087464172, 228.63874151};
  float dists[4] = {0.00676530475436, -0.000811126898338, 0.0166458761987, -0.0172655346139};

  cu::ImageUndistorter<PinholeGeometry, EquidistantDistortion, float> undistorter(752, 480, params, dists);
  undistorter.undistort(*cv_img);
}
