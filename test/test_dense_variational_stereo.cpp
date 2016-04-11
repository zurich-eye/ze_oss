#include <assert.h>
#include <cstdint>
#include <iostream>
#include <memory>
#include <tuple>

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

//-------------------------------------------------------------------------------------------------
// parameters: solver, scale_factor, expected error
class DenseStereoTests
    : public ::testing::TestWithParam<std::tuple<ze::cu::StereoPDSolver, double, double>>
{ };

//-------------------------------------------------------------------------------------------------
TEST_P(DenseStereoTests, StereoAlgorithms)
{
  using namespace ze::cu;
  using Stereo = ze::cu::VariationalStereo;
  using StereoParameters = Stereo::Parameters;

  // Load two images:
  std::string data_path = ze::getTestDataDir("computer_vision_images");
  ImageGpu32fC1::Ptr cuimg_ref, cuimg_cur;
  cvBridgeLoad(cuimg_ref, data_path + "/cones/im2.png", ze::PixelOrder::gray);
  cvBridgeLoad(cuimg_cur, data_path + "/cones/im6.png", ze::PixelOrder::gray);
//  ze::cvBridgeLoad(gt_disp_ref, data_path + "/cones/disp2.png", ze::PixelOrder::gray);
//  ze::cvBridgeLoad(gt_disp_cur, data_path + "/cones/disp6.png", ze::PixelOrder::gray);

  {
    ze::Pixel32fC1 min_val,max_val;
    minMax(*cuimg_ref, min_val, max_val);
    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
  }

  // compute dense stereo
  StereoParameters::Ptr stereo_params = std::make_shared<StereoParameters>();
  stereo_params->solver = std::get<0>(GetParam());
  stereo_params->ctf.scale_factor = std::get<1>(GetParam());;
  stereo_params->ctf.iters = 100;
  stereo_params->ctf.warps  = 10;
  stereo_params->ctf.apply_median_filter = true;

  std::unique_ptr<Stereo> stereo(new Stereo(stereo_params));

  stereo->addImage(cuimg_ref);
  stereo->addImage(cuimg_cur);

  stereo->solve();

  ImageGpu32fC1::Ptr cudisp = stereo->getDisparities();
  CHECK_NOTNULL(cudisp.get());
  ImageGpu32fC1::Ptr cuocc = stereo->getOcclusion();

  {
    ze::Pixel32fC1 min_val,max_val;
    minMax(*cudisp, min_val, max_val);
    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
  }


  if (FLAGS_visualize)
  {
    std::stringstream ss_window_name;
    ss_window_name << "Disparities: ";
    switch (stereo_params->solver)
    {
    case StereoPDSolver::HuberL1:
      ss_window_name << "HL1";
      break;
    case StereoPDSolver::PrecondHuberL1:
      ss_window_name << "HL1 (precond.)";
      break;
    case StereoPDSolver::PrecondHuberL1Weighted:
      ss_window_name << "HL1-weighted (precond.)";
      break;
    default:
      ss_window_name << "unknown solver";
      break;
    }
    ss_window_name << ", " << stereo_params->ctf.scale_factor;

    cvBridgeShow("Reference Image", *cuimg_ref);
    cvBridgeShow("Current Image", *cuimg_cur);
    cvBridgeShow(ss_window_name.str(), *cudisp, true);
    if (cuocc)
    {
      cvBridgeShow("Occlusions", *cuocc);
    }
    cv::waitKey(0);
  }
}

//-------------------------------------------------------------------------------------------------
std::tuple<ze::cu::StereoPDSolver, double, double> const StereoTestsParametrizationTable[] = {
  //          solver                                           scale_factor     error
  std::make_tuple( ze::cu::StereoPDSolver::HuberL1,                 0.5,             0.0),
  std::make_tuple( ze::cu::StereoPDSolver::PrecondHuberL1,          0.5,             0.0),
  std::make_tuple( ze::cu::StereoPDSolver::PrecondHuberL1Weighted,  0.5,             0.0),
  //          solver                                           scale_factor     error
  std::make_tuple( ze::cu::StereoPDSolver::HuberL1,                 0.8,             0.0),
  std::make_tuple( ze::cu::StereoPDSolver::PrecondHuberL1,          0.8,             0.0),
  std::make_tuple( ze::cu::StereoPDSolver::PrecondHuberL1Weighted,  0.8,             0.0),
  //          solver                                           scale_factor     error
  std::make_tuple( ze::cu::StereoPDSolver::HuberL1,                 0.95,            0.0),
  std::make_tuple( ze::cu::StereoPDSolver::PrecondHuberL1,          0.95,            0.0),
  std::make_tuple( ze::cu::StereoPDSolver::PrecondHuberL1Weighted,  0.95,            0.0),
};

//-------------------------------------------------------------------------------------------------
INSTANTIATE_TEST_CASE_P(
  DenseStereoSolverTests, DenseStereoTests, ::testing::ValuesIn(StereoTestsParametrizationTable));


ZE_UNITTEST_ENTRYPOINT
