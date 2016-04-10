#include <assert.h>
#include <cstdint>
#include <iostream>
#include <memory>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
//#include <ze/common/benchmark.h>
//#include <ze/common/matrix.h>
//#include <ze/common/timer.h>
//#include <ze/cameras/camera.h>
//#include <ze/cameras/camera_rig.h>
//#include <ze/cameras/camera_utils.h>

#include <imp/core/roi.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_math.cuh>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>

//#include <imp/cu_core/cu_se3.cuh>
//#include <imp/cu_core/cu_matrix.cuh>
//#include <imp/cu_core/cu_pinhole_camera.cuh>

#include <imp/cu_correspondence/variational_stereo.hpp>

DEFINE_bool(visualize, false, "Show input images and results");

TEST(DenseStereoTests, testVariationalStereoHuberL1)
{
  using Stereo = ze::cu::VariationalStereo;
  using StereoParameters = Stereo::Parameters;

  // Load two images:
  std::string data_path = ze::getTestDataDir("computer_vision_images");
  ze::cu::ImageGpu32fC1::Ptr cuimg_ref, cuimg_cur;
  ze::cu::cvBridgeLoad(cuimg_ref, data_path + "/cones/im2.png", ze::PixelOrder::gray);
  ze::cu::cvBridgeLoad(cuimg_cur, data_path + "/cones/im6.png", ze::PixelOrder::gray);
//  ze::cvBridgeLoad(gt_disp_ref, data_path + "/cones/disp2.png", ze::PixelOrder::gray);
//  ze::cvBridgeLoad(gt_disp_cur, data_path + "/cones/disp6.png", ze::PixelOrder::gray);

  {
    ze::Pixel32fC1 min_val,max_val;
    ze::cu::minMax(*cuimg_ref, min_val, max_val);
    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
  }

  // compute dense stereo
  StereoParameters::Ptr stereo_params = std::make_shared<StereoParameters>();
  stereo_params->solver = ze::cu::StereoPDSolver::HuberL1;
  stereo_params->ctf.scale_factor = 0.8f;
  stereo_params->ctf.iters = 50;
  stereo_params->ctf.warps  = 10;
  stereo_params->ctf.apply_median_filter = true;

  std::unique_ptr<Stereo> stereo(new Stereo(stereo_params));

  stereo->addImage(cuimg_ref);
  stereo->addImage(cuimg_cur);

  stereo->solve();

  ze::cu::ImageGpu32fC1::Ptr cudisp = stereo->getDisparities();
  ze::cu::ImageGpu32fC1::Ptr cuocc = stereo->getOcclusion();

  CHECK_NOTNULL(cudisp.get());

  {
    ze::Pixel32fC1 min_val,max_val;
    ze::cu::minMax(*cudisp, min_val, max_val);
    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
  }


  if (FLAGS_visualize)
  {
    ze::cu::cvBridgeShow("Reference Image", *cuimg_ref);
    ze::cu::cvBridgeShow("Current Image", *cuimg_cur);
    ze::cu::cvBridgeShow("Disparities Huber-L1", *cudisp, true);
    if (cuocc)
    {
      ze::cu::cvBridgeShow("Occlusions", *cuocc);
    }
    cv::waitKey(0);
  }
}

//-----------------------------------------------------------------------------
TEST(DenseStereoTests, testVariationalStereoPrecondHuberL1)
{
  using Stereo = ze::cu::VariationalStereo;
  using StereoParameters = Stereo::Parameters;

  // Load two images:
  std::string data_path = ze::getTestDataDir("computer_vision_images");
  ze::cu::ImageGpu32fC1::Ptr cuimg_ref, cuimg_cur;
  ze::cu::cvBridgeLoad(cuimg_ref, data_path + "/cones/im2.png", ze::PixelOrder::gray);
  ze::cu::cvBridgeLoad(cuimg_cur, data_path + "/cones/im6.png", ze::PixelOrder::gray);
//  ze::cvBridgeLoad(gt_disp_ref, data_path + "/cones/disp2.png", ze::PixelOrder::gray);
//  ze::cvBridgeLoad(gt_disp_cur, data_path + "/cones/disp6.png", ze::PixelOrder::gray);

  {
    ze::Pixel32fC1 min_val,max_val;
    ze::cu::minMax(*cuimg_ref, min_val, max_val);
    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
  }

  // compute dense stereo
  StereoParameters::Ptr stereo_params = std::make_shared<StereoParameters>();
  stereo_params->solver = ze::cu::StereoPDSolver::PrecondHuberL1;
  stereo_params->ctf.scale_factor = 0.8f;
  stereo_params->ctf.iters = 50;
  stereo_params->ctf.warps  = 10;
  stereo_params->ctf.apply_median_filter = true;

  std::unique_ptr<Stereo> stereo(new Stereo(stereo_params));

  stereo->addImage(cuimg_ref);
  stereo->addImage(cuimg_cur);

  stereo->solve();

  ze::cu::ImageGpu32fC1::Ptr cudisp = stereo->getDisparities();
  ze::cu::ImageGpu32fC1::Ptr cuocc = stereo->getOcclusion();

  CHECK_NOTNULL(cudisp.get());

  {
    ze::Pixel32fC1 min_val,max_val;
    ze::cu::minMax(*cudisp, min_val, max_val);
    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
  }


  if (FLAGS_visualize)
  {
    ze::cu::cvBridgeShow("Reference Image", *cuimg_ref);
    ze::cu::cvBridgeShow("Current Image", *cuimg_cur);
    ze::cu::cvBridgeShow("Disparities Preconditioned Huber-L1", *cudisp, true);
    if (cuocc)
    {
      ze::cu::cvBridgeShow("Occlusions", *cuocc);
    }
    cv::waitKey(0);
  }
}

//-----------------------------------------------------------------------------
TEST(DenseStereoTests, testVariationalStereoPrecondHuberL1Weighted)
{
  using Stereo = ze::cu::VariationalStereo;
  using StereoParameters = Stereo::Parameters;

  // Load two images:
  std::string data_path = ze::getTestDataDir("computer_vision_images");
  ze::cu::ImageGpu32fC1::Ptr cuimg_ref, cuimg_cur;
  ze::cu::cvBridgeLoad(cuimg_ref, data_path + "/cones/im2.png", ze::PixelOrder::gray);
  ze::cu::cvBridgeLoad(cuimg_cur, data_path + "/cones/im6.png", ze::PixelOrder::gray);
//  ze::cvBridgeLoad(gt_disp_ref, data_path + "/cones/disp2.png", ze::PixelOrder::gray);
//  ze::cvBridgeLoad(gt_disp_cur, data_path + "/cones/disp6.png", ze::PixelOrder::gray);

  {
    ze::Pixel32fC1 min_val,max_val;
    ze::cu::minMax(*cuimg_ref, min_val, max_val);
    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
  }

  // compute dense stereo
  StereoParameters::Ptr stereo_params = std::make_shared<StereoParameters>();
  stereo_params->solver = ze::cu::StereoPDSolver::PrecondHuberL1Weighted;
  stereo_params->ctf.scale_factor = 0.8f;
  stereo_params->ctf.iters = 50;
  stereo_params->ctf.warps  = 10;
  stereo_params->ctf.apply_median_filter = true;

  std::unique_ptr<Stereo> stereo(new Stereo(stereo_params));

  stereo->addImage(cuimg_ref);
  stereo->addImage(cuimg_cur);

  stereo->solve();

  ze::cu::ImageGpu32fC1::Ptr cudisp = stereo->getDisparities();
  ze::cu::ImageGpu32fC1::Ptr cuocc = stereo->getOcclusion();

  CHECK_NOTNULL(cudisp.get());

  {
    ze::Pixel32fC1 min_val,max_val;
    ze::cu::minMax(*cudisp, min_val, max_val);
    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
  }


  if (FLAGS_visualize)
  {
    ze::cu::cvBridgeShow("Reference Image", *cuimg_ref);
    ze::cu::cvBridgeShow("Current Image", *cuimg_cur);
    ze::cu::cvBridgeShow("Disparities Preconditioned Huber-L1 Edge Weighted", *cudisp, true);
    if (cuocc)
    {
      ze::cu::cvBridgeShow("Occlusions", *cuocc);
    }
    cv::waitKey(0);
  }
}


ZE_UNITTEST_ENTRYPOINT
