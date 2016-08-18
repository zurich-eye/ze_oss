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

TEST(TrajectorySimulator, testSplineScenario)
{
  using namespace ze;

  // Create trajectory:
  PoseSeries pose_series;
  pose_series.load(joinPath(getTestDataDir("ze_applanix_gt_data"), "traj_gt.csv"));
  StampedTransformationVector poses = pose_series.getStampedTransformationVector();

  std::shared_ptr<BSplinePoseMinimalRotationVector> bs =
      std::make_shared<BSplinePoseMinimalRotationVector>(3);
  bs->initPoseSplinePoses(poses, 100, 0.5);
  TrajectorySimulator::Ptr trajectory = std::make_shared<SplineTrajectorySimulator>(bs);

  // Create camera:
  CameraRig::Ptr rig = cameraRigFromYaml(joinPath(getTestDataDir("camera_models"),
                                                  "camera_rig_1.yaml"));

  // Create visualizer:
  Visualizer::Ptr visualizer = std::make_shared<VisualizerRos>();

  // Create camera simulator:
  CameraSimulatorOptions options;
  options.min_depth = 4.0;
  options.max_depth = 10.0;
  options.max_num_landmarks_ = 20000;
  CameraSimulator cam_sim(trajectory, rig, options);
  cam_sim.setVisualizer(visualizer);
  cam_sim.initializeMap();
  for (int i = 0; i < 1000; ++i)
  {
    cam_sim.visualize(1.0, 4.0, 0.3);
  }
}

ZE_UNITTEST_ENTRYPOINT
