#include <assert.h>
#include <cstdint>
#include <iostream>
#include <memory>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ze/common/benchmark.h>
#include <ze/common/matrix.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/common/timer.h>
#include <ze/cameras/camera.h>
#include <ze/cameras/camera_rig.h>
#include <ze/cameras/camera_utils.h>

#include <imp/core/roi.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_math.cuh>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>

#include <imp/cu_core/cu_se3.cuh>
#include <imp/cu_core/cu_matrix.cuh>
#include <imp/cu_core/cu_pinhole_camera.cuh>

#include <imp/cu_correspondence/variational_epipolar_stereo.hpp>

DEFINE_bool(visualize, false, "Show input images and results");



TEST(DenseStereoTests, testVariationalEpipolarStereo)
{
  // Load two images:
  std::string data_path = ze::getTestDataDir("synthetic_room_pinhole");
  std::shared_ptr<ze::ImageCv8uC1> img_ref, img_cur;
  ze::cvBridgeLoad(img_ref, data_path + "/img/img0001_0.png", ze::PixelOrder::gray);
  ze::cvBridgeLoad(img_cur, data_path + "/img/img0003_0.png", ze::PixelOrder::gray);

  // Load poses:
  std::map<int64_t, ze::Transformation> T_W_C_vec =
      ze::loadIndexedPosesFromCsv(data_path + "/traj_gt.csv");
  ze::Transformation T_ref_cur = T_W_C_vec[1].inverse() * T_W_C_vec[3];

  // Load camera:
  ze::Camera::Ptr cam = ze::Camera::loadFromYaml(data_path + "/calib.yaml");

  // Load depthmap from reference image:
  ze::ImageRaw32fC1 depth_ref(cam->width(), cam->height());
  CHECK_EQ(depth_ref.width(), depth_ref.stride());
  ze::loadDepthmapFromFile(data_path + "/depth/img0001_0.depth",
                           depth_ref.numel(),
                           reinterpret_cast<float*>(depth_ref.data()));

  // init gpu instances
//  ze::cu::ImageGpu32fC1::Ptr cuimg_ref = std::make_shared<ze::cu::ImageGpu32fC1>(*img_ref);
//  ze::cu::ImageGpu32fC1::Ptr cuimg_cur = std::make_shared<ze::cu::ImageGpu32fC1>(*img_cur);






  if (FLAGS_visualize)
  {
    ze::cvBridgeShow("ref_im", ze::ImageCv8uC1(*img_ref));
    ze::cvBridgeShow("cur_im", ze::ImageCv8uC1(*img_cur));
    cv::waitKey(0);
  }
}



ZE_UNITTEST_ENTRYPOINT
