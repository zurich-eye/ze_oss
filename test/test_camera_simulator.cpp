#include <ze/vi_simulation/camera_simulator.hpp>
#include <ze/vi_simulation/trajectory_simulator.hpp>
#include <ze/cameras/camera_rig.h>
#include <ze/common/csv_trajectory.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/common/types.h>
#include <ze/common/path_utils.h>
#include <ze/common/random_matrix.hpp>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/visualization/viz_ros.h>
#ifdef ZE_USE_OPENCV
#include <opencv2/highgui/highgui.hpp>
#endif

TEST(CameraSimulator, testSplineScenario)
{
  using namespace ze;

  // Create trajectory:
  PoseSeries pose_series;
  pose_series.load(joinPath(getTestDataDir("ze_applanix_gt_data"), "traj_es.csv"));
  StampedTransformationVector poses = pose_series.getStampedTransformationVector();

  std::shared_ptr<BSplinePoseMinimalRotationVector> bs =
      std::make_shared<BSplinePoseMinimalRotationVector>(3);
  bs->initPoseSplinePoses(poses, 100, 0.5);
  TrajectorySimulator::Ptr trajectory = std::make_shared<SplineTrajectorySimulator>(bs);

  // Create camera:
  CameraRig::Ptr rig = cameraRigFromYaml(joinPath(getTestDataDir("camera_models"),
                                                  "camera_rig_3.yaml"));

  // Create visualizer:
  Visualizer::Ptr visualizer = std::make_shared<VisualizerRos>();

  // Create camera simulator:
  CameraSimulatorOptions options;
  options.min_depth_m = 4.0;
  options.max_depth_m = 10.0;
  options.max_num_landmarks_ = 20000;
  CameraSimulator cam_sim(trajectory, rig, options);
  cam_sim.setVisualizer(visualizer);
  cam_sim.initializeMap();
  for (int i = 0; i < 100; ++i)
  {
    cam_sim.visualize(1.0, 4.0, 0.3);
  }

#ifdef ZE_USE_OPENCV
  cv::Mat img_0(rig->at(0).height(), rig->at(0).width(), CV_8UC1, cv::Scalar(0));
#endif

  // Loop through trajectory and visualize feature tracks:
  for (int j = 0; j < 1000; ++j)
  {
    real_t time = cam_sim.trajectory().start() + j * 1.0/20.0;
    const CameraMeasurementsVector& m_vec = cam_sim.getMeasurements(time);
#ifdef ZE_USE_OPENCV
    if (false)
    {
      const CameraMeasurements& m = m_vec[0];
      for (int i = 0; i < m.keypoints_.cols(); ++i)
      {
        cv::circle(img_0, cv::Point(m.keypoints_(0,i), m.keypoints_(1,i)), 3,
                   cv::Scalar(m.local_track_ids_[i] % 255), 3);
      }
      cv::imshow("img_0", img_0);
      cv::waitKey(1);
    }
#endif
  }

  VLOG(1) << "Timing results: \n" << cam_sim.timer_;
}

ZE_UNITTEST_ENTRYPOINT
