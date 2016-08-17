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

#include <imp/core/roi.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_math.cuh>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>

#include <imp/cu_correspondence/variational_stereo.hpp>

DEFINE_bool(visualize, false, "Show input images and results.");
DEFINE_bool(writePFM, true, "Write disparity output as PFM file.");

//-----------------------------------------------------------------------------
// check whether machine is little endian
int littleendian()
{
    int intval = 1;
    uchar *uval = (uchar *)&intval;
    return uval[0] == 1;
}

//-----------------------------------------------------------------------------
// write pfm image
// 1-band PFM image, see http://netpbm.sourceforge.net/doc/pfm.html
void writePFM(const ze::ImageRaw32fC1& disp, const std::string& filename,
              float scalefactor=1.f/255.f)
{
  // Open the file
  FILE *stream = fopen(filename.c_str(), "wb");
  CHECK(stream != 0) << "writePFM could not open File " << filename;

  // sign of scalefact indicates endianness, see pfms specs
  if (littleendian())
  {
    scalefactor = -scalefactor;
  }

  // write the header: 3 lines: Pf, dimensions, scale factor (negative val == little endian)
  fprintf(stream, "Pf\n%d %d\n%f\n", disp.width(), disp.height(), scalefactor);


  for (uint32_t y = disp.height(); y-- > 0;)
  {
    for (uint32_t x = 0; x < disp.width(); ++x)
    {
      float disp_value = static_cast<float>(disp(x,y));
      if (disp_value > 0.f)
      {
        disp_value = INFINITY;
      }
      CHECK(1 == static_cast<uint32_t>(
              fwrite(&disp_value, sizeof(float), 1, stream)));
    }
  }
  fclose(stream);
}

//-----------------------------------------------------------------------------
// parameters: solver, scale_factor, expected error
class DenseStereoTests
    : public ::testing::TestWithParam<std::tuple<ze::cu::StereoPDSolver, double, double>>
{ };

//-----------------------------------------------------------------------------
TEST_P(DenseStereoTests, StereoAlgorithms)
{
  using namespace ze::cu;
  using Stereo = ze::cu::VariationalStereo;
  using StereoParameters = Stereo::Parameters;

  // Load two images:
  std::string data_path = ze::getTestDataDir("computer_vision_images");
  ImageGpu32fC1::Ptr cuimg_ref, cuimg_cur;
  cvBridgeLoad(cuimg_ref, data_path + "/middlebury/trainingQ/Teddy/im0.png", ze::PixelOrder::gray);
  cvBridgeLoad(cuimg_cur, data_path + "/middlebury/trainingQ/Teddy/im1.png", ze::PixelOrder::gray);

  // compute dense stereo
  StereoParameters::Ptr stereo_params = std::make_shared<StereoParameters>();
  stereo_params->solver = std::get<0>(GetParam());
  stereo_params->ctf.scale_factor = std::get<1>(GetParam());;
  //  stereo_params->ctf.iters = 100;
  //  stereo_params->ctf.warps  = 10;
  //  stereo_params->ctf.apply_median_filter = true;

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


  if (FLAGS_writePFM)
  {
    std::stringstream ss_pfm_filename;
    ss_pfm_filename << data_path << "/middlebury/trainingQ/Teddy/disp1";
    std::string algorithm_short_string="?";
    switch (stereo_params->solver)
    {
    case StereoPDSolver::HuberL1:
      ss_pfm_filename << "HL1";
      break;
    case StereoPDSolver::PrecondHuberL1:
      ss_pfm_filename << "HL1p";
      break;
    case StereoPDSolver::PrecondHuberL1Weighted:
      ss_pfm_filename << "HL1p-w";
      break;
    default:
      ss_pfm_filename << "unknown";
      break;
    }
    ss_pfm_filename << "_sf";
    ss_pfm_filename << stereo_params->ctf.scale_factor;
    ss_pfm_filename << ".pfm";
    std::string pfm_filename = ss_pfm_filename.str();

    ze::ImageRaw32fC1::Ptr disp = std::make_shared<ze::ImageRaw32fC1>(*cudisp);
    VLOG(1) << "Writing disparities to '" << pfm_filename << "'.";
    writePFM(*disp, pfm_filename, 1.f/55.f);
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

//-----------------------------------------------------------------------------
//! @todo (MWE) add desired error to make tests more useful.
using Solver = ze::cu::StereoPDSolver;
std::tuple<ze::cu::StereoPDSolver, double, double> const
StereoTestsParametrizationTable[] =
{
  //              solver                           scale_factor  error
  std::make_tuple(Solver::HuberL1,                 0.5,          0.0),
  std::make_tuple(Solver::PrecondHuberL1,          0.5,          0.0),
  std::make_tuple(Solver::PrecondHuberL1Weighted,  0.5,          0.0),
  //              solver                           scale_factor  error
  std::make_tuple(Solver::HuberL1,                 0.8,          1.23),
  std::make_tuple(Solver::PrecondHuberL1,          0.8,          1.23),
  std::make_tuple(Solver::PrecondHuberL1Weighted,  0.8,          1.33),
  //              solver                           scale_factor  error
  std::make_tuple(Solver::HuberL1,                 0.95,         0.0),
  std::make_tuple(Solver::PrecondHuberL1,          0.95,         0.0),
  std::make_tuple(Solver::PrecondHuberL1Weighted,  0.95,         0.0),
};

//-----------------------------------------------------------------------------
INSTANTIATE_TEST_CASE_P(
    DenseStereoSolverTests, DenseStereoTests,
    ::testing::ValuesIn(StereoTestsParametrizationTable));


ZE_UNITTEST_ENTRYPOINT
