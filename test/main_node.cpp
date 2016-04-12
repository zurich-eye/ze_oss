#include <glog/logging.h>
#include <gflags/gflags.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

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

#include <ze/cameras/camera.h>
#include <ze/cameras/camera_rig.h>

//#include <ze/common/file_utils.h>

DEFINE_string(calibration_file, "", "Path to input calibration YAML file");

using Stereo = ze::cu::VariationalEpipolarStereo;
using StereoParameters = Stereo::Parameters;

namespace ze {
class StereoNode {

public:
  StereoNode() = delete;
  ~StereoNode() = default;
  StereoNode(std::unique_ptr<Stereo>& stereo)
    : stereo_(stereo)
  { }

  void callback(const sensor_msgs::ImageConstPtr& img0,
                const sensor_msgs::ImageConstPtr& img1)
  {
    VLOG(1) << "received stereo pair";
    cv::Mat mat0_8uc1, mat1_8uc1;
    try
    {
      mat0_8uc1 = cv_bridge::toCvShare(img0, sensor_msgs::image_encodings::MONO8)->image;
      mat1_8uc1 = cv_bridge::toCvShare(img1, sensor_msgs::image_encodings::MONO8)->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat mat0_32fc1, mat1_32fc1;
    mat0_8uc1.convertTo(mat0_32fc1, CV_32F, 1.0f/255.0f);
    mat1_8uc1.convertTo(mat1_32fc1, CV_32F, 1.0f/255.0f);

    ze::Size2u im_size((std::uint32_t)mat0_32fc1.cols, (std::uint32_t)mat0_32fc1.rows);
    if (!cv_img0_ || !cv_img1_ || !cu_img0_ || !cu_img1_ || im_size != cv_img0_->size())
    {
      cv_img0_.reset(new ze::ImageCv32fC1(im_size));
      cv_img1_.reset(new ze::ImageCv32fC1(im_size));
      cu_img0_.reset(new ze::cu::ImageGpu32fC1(im_size));
      cu_img1_.reset(new ze::cu::ImageGpu32fC1(im_size));
    }
    cv_img0_->cvMat() = mat0_32fc1;
    cv_img1_->cvMat() = mat1_32fc1;
    cu_img0_->copyFrom(*cv_img0_);
    cu_img1_->copyFrom(*cv_img1_);

    stereo_->addImage(cu_img0_);
    stereo_->addImage(cu_img1_);
    stereo_->solve();

    //cv::imshow("cv_img0", cv_img0_->cvMat());
    //cv::waitKey(1);
  }
private:
  ze::ImageCv32fC1::Ptr cv_img0_;
  ze::ImageCv32fC1::Ptr cv_img1_;
  ze::cu::ImageGpu32fC1::Ptr cu_img0_;
  ze::cu::ImageGpu32fC1::Ptr cu_img1_;
  std::unique_ptr<Stereo>& stereo_;
};
} // ze namespace

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "ze_stereo_node");
  ros::NodeHandle nh;

  ze::CameraRig::Ptr rig = ze::CameraRig::loadFromYaml(FLAGS_calibration_file);
  ze::Camera::Ptr cam0 = rig->atShared(0);
  ze::Camera::Ptr cam1 = rig->atShared(1);

  const ze::VectorX projection_parameteres0 = cam0->projectionParameters();
  ze::cu::PinholeCamera cu_cam0(projection_parameteres0(0), projection_parameteres0(1),
                                projection_parameteres0(2), projection_parameteres0(3));
  const ze::VectorX projection_parameteres1 = cam1->projectionParameters();
  ze::cu::PinholeCamera cu_cam1(projection_parameteres1(0), projection_parameteres1(1),
                                projection_parameteres1(2), projection_parameteres1(3));

  ze::Transformation T_ref_cur = rig->get_T_C_B(0) * rig->get_T_C_B(1).inverse();

  ze::cu::Matrix3f F_ref_cur;
  ze::cu::Matrix3f F_cur_ref;
  Eigen::Matrix3d F_fm, F_mf;
  { // compute fundamental matrix
    Eigen::Matrix3d R_ref_cur = T_ref_cur.getRotationMatrix();

    // in ref coordinates
    Eigen::Vector3d t_ref_cur = T_ref_cur.getPosition();

    Eigen::Matrix3d tx_ref_cur;
    tx_ref_cur << 0, -t_ref_cur[2], t_ref_cur[1],
        t_ref_cur[2], 0, -t_ref_cur[0],
        -t_ref_cur[1], t_ref_cur[0], 0;
    Eigen::Matrix3d E_ref_cur = tx_ref_cur * R_ref_cur;
    Eigen::Matrix3d K0, K1;
    K0 << cu_cam0.fx(), 0, cu_cam0.cx(),
        0, cu_cam0.fy(), cu_cam0.cy(),
        0, 0, 1;
    K1 << cu_cam1.fx(), 0, cu_cam1.cx(),
        0, cu_cam1.fy(), cu_cam1.cy(),
        0, 0, 1;


    F_fm = K0.inverse().transpose() * E_ref_cur * K1.inverse().transpose();
    F_mf = F_fm.transpose();

    // convert the Eigen-thingy to something that we can use in CUDA
    for(size_t row=0; row<F_ref_cur.rows(); ++row)
    {
      for(size_t col=0; col<F_ref_cur.cols(); ++col)
      {
        F_ref_cur(row,col) = (float)F_fm(row,col);
        F_cur_ref(row,col) = (float)F_mf(row,col);
      }
    }
  } // end .. compute fundamental matrix

  //! @todo (mwe) this also needs to get simpler from cpu transformation to gpu transformation...
  //  Eigen::Quaterniond q_ref_cur = T_ref_cur.getEigenQuaternion();
  Eigen::Quaterniond q_cur_ref = T_ref_cur.inverse().getEigenQuaternion();
  Eigen::Vector3d t_cur_ref = T_ref_cur.inverse().getPosition();

  ze::cu::SE3<float> cu_T_cur_ref(
        static_cast<float>(q_cur_ref.w()), static_cast<float>(q_cur_ref.x()),
        static_cast<float>(q_cur_ref.y()), static_cast<float>(q_cur_ref.z()),
        static_cast<float>(t_cur_ref.x()), static_cast<float>(t_cur_ref.y()),
        static_cast<float>(t_cur_ref.z()));

  VLOG(300) << "(gpu) T_cur_ref: " << cu_T_cur_ref;

  // compute dense stereo
  StereoParameters::Ptr stereo_params = std::make_shared<StereoParameters>();
  stereo_params->solver = ze::cu::StereoPDSolver::EpipolarPrecondHuberL1;
  stereo_params->ctf.scale_factor = 0.8;
  stereo_params->ctf.iters = 100;
  stereo_params->ctf.warps  = 10;
  stereo_params->ctf.apply_median_filter = true;
  //stereo->parameters()->lambda = 20;

  std::unique_ptr<Stereo> stereo(new Stereo(stereo_params));

  stereo->setFundamentalMatrix(F_cur_ref);
  stereo->setIntrinsics({cu_cam0, cu_cam1});
  stereo->setExtrinsics(cu_T_cur_ref);

  /*
  ImageGpu32fC1::Ptr cudisp = stereo->getDisparities();
  CHECK_NOTNULL(cudisp.get());
  ImageGpu32fC1::Ptr cuocc = stereo->getOcclusion();

  {
    ze::Pixel32fC1 min_val,max_val;
    minMax(*cudisp, min_val, max_val);
    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
  }
  */

  ze::StereoNode stereo_node(stereo);

  message_filters::Subscriber<sensor_msgs::Image> img0_sub(nh, "/cam0/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> img1_sub(nh, "/cam1/image_raw", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(img0_sub, img1_sub, 1);
  sync.registerCallback(boost::bind(&ze::StereoNode::callback, &stereo_node, _1, _2));

  ros::spin();

  return EXIT_SUCCESS;
}
