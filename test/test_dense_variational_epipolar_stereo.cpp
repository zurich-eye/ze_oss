#include <assert.h>
#include <cstdint>
#include <iostream>
#include <memory>
#include <tuple>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/core/roi.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_math.cuh>
#include <imp/cu_core/cu_matrix.cuh>
#include <imp/cu_core/cu_pinhole_camera.cuh>
#include <imp/cu_core/cu_se3.cuh>
#include <imp/cu_correspondence/variational_epipolar_stereo.hpp>
#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/logging.hpp>
#include <ze/geometry/epipolar_geometry.hpp>

DEFINE_bool(visualize, false, "Show input images and results");

//-----------------------------------------------------------------------------
// parameters: solver, scale_factor, expected error
class DenseEpipolarStereoTests
    : public ::testing::TestWithParam<std::tuple<ze::cu::StereoPDSolver, double, double>>
{ };

//-----------------------------------------------------------------------------
TEST_P(DenseEpipolarStereoTests, EpipolarStereoAlgorithms)
{
  using namespace ze::cu;
  using Stereo = ze::cu::VariationalEpipolarStereo;
  using StereoParameters = Stereo::Parameters;

  // Load two images:
  std::string data_path = ze::getTestDataDir("synthetic_room_pinhole");
  ImageGpu32fC1::Ptr cuimg_ref, cuimg_cur;
  cvBridgeLoad(cuimg_ref, data_path + "/img/1.png", ze::PixelOrder::gray);
  cvBridgeLoad(cuimg_cur, data_path + "/img/3.png", ze::PixelOrder::gray);

  // Load poses:
  std::map<int64_t, ze::Transformation> T_W_C_vec =
      ze::loadIndexedPosesFromCsv(data_path + "/traj_gt.csv");
  ze::Transformation T_ref_cur = T_W_C_vec[1].inverse() * T_W_C_vec[3];

  VLOG(10) << "load camera";
  ze::Camera::Ptr cam = ze::cameraFromYaml(data_path + "/calib.yaml");

  const ze::VectorX projection_parameteres = cam->projectionParameters();
  ze::cu::PinholeCamera cu_cam(projection_parameteres(0), projection_parameteres(1),
                               projection_parameteres(2), projection_parameteres(3));

  ze::ImageRaw32fC1 depth_ref(cam->width(), cam->height());
  CHECK_EQ(depth_ref.width(), depth_ref.stride());
  ze::loadDepthmapFromFile(data_path + "/depth/1.depth",
                           depth_ref.numel(),
                           reinterpret_cast<float*>(depth_ref.data()));

  VLOG(10) << "compute fundamental matrix";

  ze::Matrix3 F_ref_cur = ze::fundamentalMatrix(T_ref_cur, projection_parameteres,
                                                projection_parameteres);
  ze::cu::Matrix3f cu_F_cur_ref(F_ref_cur.transpose());

  //! @todo (mwe) this also needs to get simpler from cpu transformation to gpu transformation...
  ze::Quaternion q_cur_ref = T_ref_cur.inverse().getRotation();
  ze::Vector3 t_cur_ref = T_ref_cur.inverse().getPosition();

  ze::cu::SE3<float> cu_T_cur_ref(
        static_cast<float>(q_cur_ref.w()), static_cast<float>(q_cur_ref.x()),
        static_cast<float>(q_cur_ref.y()), static_cast<float>(q_cur_ref.z()),
        static_cast<float>(t_cur_ref.x()), static_cast<float>(t_cur_ref.y()),
        static_cast<float>(t_cur_ref.z()));

  VLOG(300) << "(gpu) T_cur_ref: " << cu_T_cur_ref;

  // compute dense stereo
  StereoParameters::Ptr stereo_params = std::make_shared<StereoParameters>();
  stereo_params->solver = std::get<0>(GetParam());
  stereo_params->ctf.scale_factor = std::get<1>(GetParam());
  stereo_params->ctf.iters = 100;
  stereo_params->ctf.warps  = 10;
  stereo_params->ctf.apply_median_filter = true;
  //stereo->parameters()->lambda = 20;

  std::unique_ptr<Stereo> stereo(new Stereo(stereo_params));

  stereo->addImage(cuimg_ref);
  stereo->addImage(cuimg_cur);

  stereo->setFundamentalMatrix(cu_F_cur_ref);
  stereo->setIntrinsics({cu_cam, cu_cam});
  stereo->setExtrinsics(cu_T_cur_ref);

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
    case StereoPDSolver::EpipolarPrecondHuberL1:
      ss_window_name << "Epipolar HL1 (precond.)";
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

//-----------------------------------------------------------------------------
//! @todo (MWE) add desired error to make tests more useful.
using Solver = ze::cu::StereoPDSolver;
std::tuple<ze::cu::StereoPDSolver, double, double>
const EpipolarStereoTestsParametrizationTable[] = {
//  //              solver                           scale_factor  error
//  std::make_tuple(Solver::EpipolarPrecondHuberL1,  0.5,          0.0),
  //              solver                           scale_factor  error
  std::make_tuple(Solver::EpipolarPrecondHuberL1,  0.8,          0.0),
  //              solver                           scale_factor  error
  std::make_tuple(Solver::EpipolarPrecondHuberL1,  0.95,         0.0),
};

//-----------------------------------------------------------------------------
INSTANTIATE_TEST_CASE_P(
  DenseEpipolarStereoTests, DenseEpipolarStereoTests,
    ::testing::ValuesIn(EpipolarStereoTestsParametrizationTable));


ZE_UNITTEST_ENTRYPOINT
