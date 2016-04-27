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
#include <ze/data_provider/data_provider_rosbag.h>
#include <ze/data_provider/camera_imu_synchronizer.h>

//#include <ze/common/file_utils.h>

DEFINE_string(calibration_file, "", "Path to input calibration YAML file");
DEFINE_string(bag_filename, "dataset.bag", "Name of bagfile in data_dir.");
DEFINE_int32(stereo_num_iterations, 50, "number of iterations per warp");
DEFINE_int32(stereo_num_warps, 5, "number of warping iterations");
DEFINE_double(stereo_lambda, 20, "tradeoff between smoothing and dataterm");
DEFINE_double(stereo_scale_factor, 0.8, "image pyramid scale factor (reduction; value in interval (0.5,1.0]");
DEFINE_string(stereo_dump_disparities_folder, "", "Path to dump disparities.");
DEFINE_bool(stereo_visualize, false, "");
DEFINE_double(stereo_viz_min_disp, 0, "");
DEFINE_double(stereo_viz_max_disp, 0, "");
namespace ze {

using Stereo = ze::cu::VariationalEpipolarStereo;
using StereoParameters = Stereo::Parameters;

//-----------------------------------------------------------------------------
class StereoNode {

public:
  StereoNode() = delete;
  ~StereoNode() = default;
  StereoNode(std::unique_ptr<Stereo>& stereo)
    : stereo_(stereo)
  { }

  void callback(const StampedImages& stamped_images,
                const ImuStamps& imu_timestamps,
                const ImuAccGyr& imu_measurements);

private:
  ze::ImageCv32fC1::Ptr cv_img0_;
  ze::ImageCv32fC1::Ptr cv_img1_;
  ze::cu::ImageGpu32fC1::Ptr cu_img0_;
  ze::cu::ImageGpu32fC1::Ptr cu_img1_;
  std::unique_ptr<Stereo>& stereo_;
};

//-----------------------------------------------------------------------------
void StereoNode::callback(const StampedImages& stamped_images,
                          const ImuStamps& imu_timestamps,
                          const ImuAccGyr& imu_measurements)
{
  VLOG(1) << "received stamped image length: " << stamped_images.size();

  VLOG(1) << "received stereo pair";

  Image8uC1::Ptr im0 = std::dynamic_pointer_cast<Image8uC1>(stamped_images.at(0).second);
  Image8uC1::Ptr im1 = std::dynamic_pointer_cast<Image8uC1>(stamped_images.at(1).second);

  CHECK_NOTNULL(im0.get());
  CHECK_NOTNULL(im1.get());

  //! @todo (MWE) hack - make proper conversion function 8bit -> 32bit
  ImageCv8uC1 cv_im0(*im0);
  ImageCv8uC1 cv_im1(*im1);

  ImageCv32fC1 cv_im0_32f(cv_im0.size());
  cv_im0.cvMat().convertTo(cv_im0_32f.cvMat(), CV_32F);
  cv_im0_32f.cvMat() /= 255.0f;

  ImageCv32fC1 cv_im1_32f(cv_im1.size());
  cv_im1.cvMat().convertTo(cv_im1_32f.cvMat(), CV_32F);
  cv_im1_32f.cvMat() /= 255.0f;

  // copy host->device
  std::shared_ptr<ze::cu::ImageGpu32fC1> cu_im0_32fC1(
        new ze::cu::ImageGpu32fC1(cv_im0_32f));
  std::shared_ptr<ze::cu::ImageGpu32fC1> cu_im1_32fC1(
        new ze::cu::ImageGpu32fC1(cv_im1_32f));

  CHECK_NOTNULL(cu_im0_32fC1.get());
  CHECK_NOTNULL(cu_im1_32fC1.get());

  stereo_->reset(); //!< @todo MWE this is a temporary hack. make sure to just have the right amount of images in your internal data structures
  stereo_->addImage(cu_im0_32fC1);
  stereo_->addImage(cu_im1_32fC1);
  stereo_->solve();
  ze::cu::ImageGpu32fC1::Ptr cu_disp = stereo_->getDisparities();
  CHECK_NOTNULL(cu_disp.get());


  {
    ze::Pixel32fC1 min_val,max_val;
    ze::cu::minMax(*cu_disp, min_val, max_val);
    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
  }

  if (FLAGS_stereo_visualize)
  {
    ze::cu::cvBridgeShow("im0", *cu_im0_32fC1);
    ze::cu::cvBridgeShow("im1", *cu_im1_32fC1);
    if (FLAGS_stereo_viz_min_disp != FLAGS_stereo_viz_max_disp)
    {
      ze::cu::cvBridgeShow("disparities", *cu_disp, FLAGS_stereo_viz_min_disp, FLAGS_stereo_viz_max_disp);
    }
    else
    {
      ze::cu::cvBridgeShow("disparities", *cu_disp, true);
    }
    cv::waitKey(1);
  }
  std::string dump_path = FLAGS_stereo_dump_disparities_folder;
  if (!dump_path.empty())
  {
    int64_t timestamp = stamped_images.at(0).first;
    std::stringstream ss;
    ss << timestamp << "_disp.png";
    VLOG(2) << "TODO save disparity map to " << ss.str();
//    ze::cvBridgeSave(ss.str(), )
  }
}

} // ze namespace

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

//  ros::init(argc, argv, "ze_stereo_node");
//  ros::NodeHandle nh;

  VLOG(1) << "load camera rig";
  ze::CameraRig::Ptr rig = ze::CameraRig::loadFromYaml(FLAGS_calibration_file);
  ze::Camera::Ptr cam0 = rig->atShared(0);
  ze::Camera::Ptr cam1 = rig->atShared(1);

  const ze::VectorX projection_parameteres0 = cam0->projectionParameters();
  ze::cu::PinholeCamera cu_cam0(projection_parameteres0(0), projection_parameteres0(1),
                                projection_parameteres0(2), projection_parameteres0(3));
  const ze::VectorX projection_parameteres1 = cam1->projectionParameters();
  ze::cu::PinholeCamera cu_cam1(projection_parameteres1(0), projection_parameteres1(1),
                                projection_parameteres1(2), projection_parameteres1(3));

  ze::Transformation T_ref_cur = rig->T_C_B(0) * rig->T_C_B(1).inverse();

  VLOG(1) << "compute fundamental matrix";
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
  ze::StereoParameters::Ptr stereo_params = std::make_shared<ze::StereoParameters>();
  stereo_params->solver = ze::cu::StereoPDSolver::EpipolarPrecondHuberL1;
  stereo_params->ctf.apply_median_filter = true;

  stereo_params->ctf.scale_factor = FLAGS_stereo_scale_factor;
  stereo_params->ctf.iters = FLAGS_stereo_num_iterations;
  stereo_params->ctf.warps  = FLAGS_stereo_num_warps;
  stereo_params->lambda = FLAGS_stereo_lambda;
  //  stereo_params->ctf.scale_factor = 0.8;
//  stereo_params->ctf.iters = 30;
//  stereo_params->ctf.warps  = 3;
//  stereo->parameters()->lambda = 20;

  std::unique_ptr<ze::Stereo> stereo(new ze::Stereo(stereo_params));
  stereo->setFundamentalMatrix(F_cur_ref);
  stereo->setIntrinsics({cu_cam0, cu_cam1});
  stereo->setExtrinsics(cu_T_cur_ref);

//  ImageGpu32fC1::Ptr cudisp = stereo->getDisparities();
//  CHECK_NOTNULL(cudisp.get());
//  ImageGpu32fC1::Ptr cuocc = stereo->getOcclusion();

//  {
//    ze::Pixel32fC1 min_val,max_val;
//    minMax(*cudisp, min_val, max_val);
//    VLOG(2) << "disp: min: " << min_val.x << " max: " << max_val.x;
//  }

  VLOG(1) << "Create Stereo Node.";
  ze::StereoNode stereo_node(stereo);

  VLOG(1) << "Create Data Provider and Synchronizer.";
  ze::DataProviderRosbag data_provider(
        FLAGS_bag_filename, "/imu0", { {"/cam0/image_raw", 0},
                                       {"/cam1/image_raw", 1} });

  ze::CameraImuSynchronizer sync(2, 1.0);
  sync.subscribeDataProvider(data_provider);
  sync.registerCameraImuCallback(
        std::bind(&ze::StereoNode::callback,
                  &stereo_node,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));


//  message_filters::Subscriber<sensor_msgs::Image> img0_sub(nh, "/cam0/image_raw", 1);
//  message_filters::Subscriber<sensor_msgs::Image> img1_sub(nh, "/cam1/image_raw", 1);
//  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(img0_sub, img1_sub, 1);
//  sync.registerCallback(boost::bind(&ze::StereoNode::callback, &stereo_node, _1, _2));

  VLOG(1) << "Start Processing.";
  data_provider.spin();
  VLOG(1) << "Finish Processing.";
//  vio_node.shutdown();

  return EXIT_SUCCESS;
}
