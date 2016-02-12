#include <ros/ros.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <ze/visualization/viz_publisher_ros.h>

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "viz_test_node");
  ros::NodeHandle pnh("~");

  ze::VisualizerRos visualizer(pnh);

  int i = 0;
  while(ros::ok())
  {
    visualizer.drawPoint(
          "point", i, ze::Vector3(0, 0, i), ze::Colors::DarkBlue);

    visualizer.drawLine(
          "line", i, ze::Vector3(i, 0, 0), ze::Vector3(i, 0.4, 0), ze::Colors::DarkRed);

    visualizer.drawCoordinateFrame(
          "frame", i, ze::Transformation(ze::Quaternion(), ze::Vector3(i, i, i)*0.2));


    sleep(1);
    ++i;
  }
}
