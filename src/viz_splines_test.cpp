#include <ros/ros.h>
#include <ze/common/logging.hpp>
#include <gflags/gflags.h>

#include <ze/visualization/viz_ros.h>
#include <ze/splines/viz_splines.hpp>
#include <ze/splines/bspline.hpp>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/common/types.h>

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Visualizer internally initializes ROS and creates a node handle.
  std::shared_ptr<ze::VisualizerRos> rv = std::make_shared<ze::VisualizerRos>();
  ze::SplinesVisualizer sv(rv);

  // Create a random 3d spline.
  ze::BSpline bs(3);

  ze::VectorX times = ze::VectorX::LinSpaced(500, 0, 100);
  ze::MatrixX points;
  points.resize(3, times.size());
  points.setRandom();

  bs.initSpline3(times, points, 100, 1e-5);
  sv.displaySplineTrajectory(bs, "spline", 0, ze::Colors::Green);

  // Create a random pose spline.
  ze::BSplinePoseMinimalRotationVector pbs(3);
  ze::MatrixX poses;
  poses.resize(6, times.size());
  poses.setRandom();
  poses.topRows(3) *= 10;

  pbs.initPoseSpline3(times, poses, 100, 1e-5);

  sv.displaySplineTrajectory(pbs, "poses", 0);

  // 2d Plots
  sv.plotSpline(bs, 0);

  // plot a pose spline as 6 dimensions
  sv.plotSpline(pbs);
}
