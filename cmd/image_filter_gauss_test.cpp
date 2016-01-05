#include <assert.h>
#include <cstdint>
#include <iostream>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <imp/core/roi.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
//#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_imgproc/cu_image_filter.cuh>
//#include <imp/bridge/opencv/cu_cv_bridge.hpp>

int main(int argc, char** argv)
{
  try
  {
    if (argc < 2)
    {
      std::cout << "usage: " << argv[0] << " input_image_filename";
      return EXIT_FAILURE;
    }
    std::string in_filename(argv[1]);

    imp::ImageCv8uC1 h1_lena_8uC1(cv::imread(in_filename, CV_LOAD_IMAGE_GRAYSCALE), imp::PixelOrder::gray);

    {
      // copy host->device
      std::unique_ptr<imp::cu::ImageGpu8uC1> d_lena_8uC1(
            new imp::cu::ImageGpu8uC1(h1_lena_8uC1));

      std::unique_ptr<imp::cu::ImageGpu8uC1> d_gauss_lena_8uC1(
            new imp::cu::ImageGpu8uC1(d_lena_8uC1->size()));

      imp::cu::filterGauss(*d_gauss_lena_8uC1, *d_lena_8uC1, 10.0);


      imp::ImageCv8uC1 h_gauss_lena_8uC1(*d_gauss_lena_8uC1);
      cv::imshow("lena 8u", h1_lena_8uC1.cvMat());
      cv::imshow("lena gauss 8u", h_gauss_lena_8uC1.cvMat());

    }

    {
      // 32fC1 test
      imp::ImageCv32fC1 h1_lena_32fC1(h1_lena_8uC1.size());

      h1_lena_8uC1.cvMat().convertTo(h1_lena_32fC1.cvMat(), CV_32F);
      h1_lena_32fC1.cvMat() /= 255.f;

      // copy host->device
      std::unique_ptr<imp::cu::ImageGpu32fC1> d_lena_32fC1(
            new imp::cu::ImageGpu32fC1(h1_lena_32fC1));

      std::unique_ptr<imp::cu::ImageGpu32fC1> d_gauss_lena_32fC1(
            new imp::cu::ImageGpu32fC1(d_lena_32fC1->size()));

      imp::cu::filterGauss(*d_gauss_lena_32fC1, *d_lena_32fC1, 10.0);


      imp::ImageCv32fC1 h_gauss_lena_32fC1(*d_gauss_lena_32fC1);
      cv::imshow("lena 32f", h1_lena_32fC1.cvMat());
      cv::imshow("lena gauss 32f", h_gauss_lena_32fC1.cvMat());

    }

//    // 32fC1 texture
//    {
//      std::shared_ptr<imp::cu::ImageGpu32fC1> d_lena;
//      imp::cu::cvBridgeLoad(d_lena, "/home/mwerlberger/data/std/Lena.tiff",
//                            imp::PixelOrder::gray);
//      std::shared_ptr<imp::cu::ImageGpu32fC1> d_lena_denoised(
//            new imp::cu::ImageGpu32fC1(d_lena->size()));

//      std::shared_ptr<imp::cu::Texture2D> d_tex_lena =
//          d_lena->genTexture(false, cudaFilterModeLinear);

//      imp::cu::filterGauss(*d_lena_denoised, *d_tex_lena, 10.0);

//      imp::cu::cvBridgeShow("lena input 32f texture", *d_lena);
//      imp::cu::cvBridgeShow("lena denoised 32f texture", *d_lena_denoised);
//    }
    cv::waitKey();
  }
  catch (std::exception& e)
  {
    std::cout << "[exception] " << e.what() << std::endl;
    assert(false);
  }

  return EXIT_SUCCESS;

}
