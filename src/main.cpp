#include <arrayfire.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/bridge/af/feature_detection.hpp>
#include <imp/bridge/af/image_af.hpp>
//#include <imp/imgproc/image_pyramid.h>
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

#if 0
  ze::cu::ImageGpu32fC1::Ptr in_img =
      std::make_shared<ze::cu::ImageGpu32fC1>(*cv_img);

  VLOG(1) << "loaded IMP image size: " << in_img->size();

  af::array b = af::loadImage(img_file_path);
  b /= 255.0;

  VLOG(1) << "loaded AF image dims: " << b.dims();

  af::array a = ze::cu::createFromImp(*in_img);

  // Sum the values and copy the result to the CPU:
  double a_sum = af::sum<float>(a);
  double b_sum = af::sum<float>(b);
  cv::Scalar cv_sum = cv::sum(cv_img->cvMat());

  double gt_sum = 0.0;
  for (size_t r=0; r<cv_img->height(); ++r)
  {
    for (size_t c=0; c<cv_img->width(); ++c)
    {
      gt_sum += cv_img->pixel(c, r);
    }
  }

  printf("a sum: %f\n", a_sum);
  printf("b sum: %f\n", b_sum);
  printf("CV sum: %f\n", cv_sum(0));
  printf("GT sum: %f\n", gt_sum);

  //! TODO (MPI): who takes care of freeing gpu memory?
#endif

  ze::ImageAF8uC1::Ptr im = std::make_shared<ze::ImageAF8uC1>(*cv_img);
  ze::ImagePyramid8uC1::Ptr pyr = ze::createAFImagePyramid<ze::Pixel8uC1>(im, 0.5, 5, 8);

  //ze::ImagePyramid8uC1 pyr(ze::Size2u(im->width(), im->height()));
  //pyr.emplace_back(im);

  const af::array& lvl0 = dynamic_cast<ze::ImageAF8uC1&>(pyr->at(0)).afArray();
  const af::array& half_sampled = dynamic_cast<ze::ImageAF8uC1&>(pyr->at(1)).afArray();

  printf("Printing level one of image pyramid dims: [%llu, %llu]\n", half_sampled.dims()[0], half_sampled.dims()[1]);

  af::Window wnd("AF array");
  while(!wnd.close())
    wnd.image(lvl0);

  af::features feat = af::fast(lvl0, 20.0f, 9, true, 0.05);
  printf("Features found: %lu\n", feat.getNumFeatures());


  return 0;
}
