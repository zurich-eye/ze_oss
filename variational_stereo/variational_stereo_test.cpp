#include <assert.h>
#include <cstdint>
#include <iostream>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <imp/core/roi.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_math.cuh>
#include <imp/bridge/opencv/cu_cv_bridge.hpp>

#include <imp/cu_correspondence/variational_stereo.hpp>

int main(int argc, char** argv)
{
  using Stereo = imp::cu::VariationalStereo;
  using StereoParameters = Stereo::Parameters;

  try
  {
    if (argc < 3)
    {
      std::cout << "usage: " << argv[0] << " left_image right_image" << std::endl;
      return EXIT_FAILURE;
    }
    std::string in_left(argv[1]);
    std::string in_right(argv[2]);


    imp::cu::ImageGpu32fC1::Ptr left_32fC1;
    imp::cu::cvBridgeLoad(left_32fC1, in_left, imp::PixelOrder::gray);
    imp::cu::ImageGpu32fC1::Ptr right_32fC1;
    imp::cu::cvBridgeLoad(right_32fC1, in_right, imp::PixelOrder::gray);

    {
      imp::Pixel32fC1 min_val,max_val;
      imp::cu::minMax(*left_32fC1, min_val, max_val);
      std::cout << "disp: min: " << min_val.x << " max: " << max_val.x << std::endl;
    }


    StereoParameters::Ptr stereo_params = std::make_shared<StereoParameters>();
    stereo_params->verbose = 10;
    stereo_params->solver = imp::cu::StereoPDSolver::PrecondHuberL1Weighted;
    stereo_params->ctf.scale_factor = 0.8f;
    stereo_params->ctf.iters = 50;
    stereo_params->ctf.warps  = 10;
    stereo_params->ctf.apply_median_filter = true;

    std::unique_ptr<Stereo> stereo(new Stereo(stereo_params));

    stereo->addImage(left_32fC1);
    stereo->addImage(right_32fC1);

    stereo->solve();

    imp::cu::ImageGpu32fC1::Ptr d_disp = stereo->getDisparities();
    imp::cu::ImageGpu32fC1::Ptr d_occ = stereo->getOcclusion();

    {
      imp::Pixel32fC1 min_val,max_val;
      imp::cu::minMax(*d_disp, min_val, max_val);
      std::cout << "disp: min: " << min_val.x << " max: " << max_val.x << std::endl;
    }

    imp::cu::cvBridgeShow("left image", *left_32fC1);
    imp::cu::cvBridgeShow("right image", *right_32fC1);
    *d_disp *= -1;
    {
      imp::Pixel32fC1 min_val,max_val;
      imp::cu::minMax(*d_disp, min_val, max_val);
      std::cout << "disp: min: " << min_val.x << " max: " << max_val.x << std::endl;
    }

    imp::cu::cvBridgeShow("disparities", *d_disp, true);

    if (d_occ)
    {
      imp::cu::cvBridgeShow("occlusions", *d_occ, true);
    }

    // get primal energy
    imp::cu::ImageGpu32fC1::Ptr d_ep = stereo->computePrimalEnergy();
    if (d_ep)
    {
      imp::cu::cvBridgeShow("primal energy", *d_ep, true);
      imp::Pixel32fC1 ep_min, ep_max;
      imp::cu::minMax(*d_ep, ep_min, ep_max);
      std::cout << "primal energy: min: " << ep_min.x << " max: " << ep_max.x << std::endl;
    }


    cv::waitKey();
  }
  catch (std::exception& e)
  {
    std::cout << "[exception] " << e.what() << std::endl;
    assert(false);
  }

  return EXIT_SUCCESS;

}
