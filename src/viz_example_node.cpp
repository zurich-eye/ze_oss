#include <ros/ros.h>
#include <ze/common/logging.hpp>
#include <gflags/gflags.h>

#include <ze/visualization/viz_ros.h>

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Visualizer internally initializes ROS and creates a node handle.
  ze::VisualizerRos visualizer;

  for(int i = 0; i < 100; ++i)
  {
    visualizer.drawPoint(
          "point", i, ze::Vector3(0, 0, i), ze::Colors::DarkBlue);

    visualizer.drawLine(
          "line", i, ze::Vector3(i, 0, 0), ze::Vector3(i, 0.4, 0), ze::Colors::DarkRed);

    visualizer.drawCoordinateFrame(
          "frame", i, ze::Transformation(ze::Quaternion(), ze::Vector3(i, i, i)*0.2));

    ze::Positions points(3, 4);
    points << 0, 1, 2, 3,
              1, 1, 1, 1,
              i, i, i, i;
    visualizer.drawPoints("points", 0, points, ze::Colors::Blue);

    ze::Lines lines;
    lines.push_back(std::make_pair(ze::Position(i, 0, 1), ze::Position(i, 0, 0)));
    lines.push_back(std::make_pair(ze::Position(i, 1, 1), ze::Position(i, 1, 0)));
    lines.push_back(std::make_pair(ze::Position(i, 2, 1), ze::Position(i, 2, 0)));
    lines.push_back(std::make_pair(ze::Position(i, 3, 1), ze::Position(i, 3, 0)));
    visualizer.drawLines("lines", 0, lines, ze::Colors::Green);

    ze::TransformationVector poses;
    poses.push_back(ze::Transformation(ze::Quaternion(ze::Vector3(0.1, 0, 0)), ze::Position(i, 0, 0)));
    poses.push_back(ze::Transformation(ze::Quaternion(ze::Vector3(0.2, 0, 0)), ze::Position(i, 1, 0)));
    poses.push_back(ze::Transformation(ze::Quaternion(ze::Vector3(0.3, 0, 0)), ze::Position(i, 2, 0)));
    poses.push_back(ze::Transformation(ze::Quaternion(ze::Vector3(0.4, 0, 0)), ze::Position(i, 3, 0)));
    visualizer.drawCoordinateFrames("poses", 0, poses);

    sleep(1);
  }
}
