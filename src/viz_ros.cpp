#include <ze/visualization/viz_ros.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ze/visualization/viz_ros_utils.h>

namespace ze {

VisualizerRos::VisualizerRos()
{
  // Inititialize ROS if it was not initialized before.
  if(!ros::isInitialized())
  {
    VLOG(1) << "Initializting ROS";
    int argc = 0;
    ros::init(argc, nullptr, std::string("ze_visualization"));
  }

  // Create node and subscribe.
  nh_.reset(new ros::NodeHandle("~"));
  pub_marker_.reset(new ros::Publisher(nh_->advertise<visualization_msgs::Marker>("markers", 100)));
}

VisualizerRos::VisualizerRos(const std::string& frame)
  : VisualizerRos::VisualizerRos()
{
  world_frame = frame;
}

void VisualizerRos::drawPoint(
    const std::string& ns,
    const size_t id,
    const Position& point,
    const Color& color,
    const FloatType size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::CUBE;
  m.action = 0; // add/modify
  FloatType marker_scale = size * viz_scale_;
  m.scale.x = marker_scale;
  m.scale.y = marker_scale;
  m.scale.z = marker_scale;
  m.color = getRosColor(color);
  m.pose.position = getRosPoint(point);
  pub_marker_->publish(m);
}

void VisualizerRos::drawLine(
    const std::string& ns,
    const size_t id,
    const Position& line_from,
    const Position& line_to,
    const Color& color,
    const FloatType size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_;
  m.scale.y = size * viz_scale_;
  m.scale.z = size * viz_scale_;
  m.color = getRosColor(color);
  m.points.reserve(2);
  m.points.push_back(getRosPoint(line_from));
  m.points.push_back(getRosPoint(line_to));
  pub_marker_->publish(m);
}

void VisualizerRos::drawCoordinateFrame(
    const std::string& ns,
    const size_t id,
    const Transformation& pose, // T_W_B
    const FloatType size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  const Vector3& p = pose.getPosition();

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_ * 0.05;
  m.colors.reserve(6);
  m.points.reserve(6);
  FloatType length = size * viz_scale_;
  m.points.push_back(getRosPoint(Vector3::Zero()));
  m.colors.push_back(getRosColor(Colors::Red));
  m.points.push_back(getRosPoint(Vector3::UnitX() * length));
  m.colors.push_back(getRosColor(Colors::Red));
  m.points.push_back(getRosPoint(Vector3::Zero()));
  m.colors.push_back(getRosColor(Colors::Green));
  m.points.push_back(getRosPoint(Vector3::UnitY() * length));
  m.colors.push_back(getRosColor(Colors::Green));
  m.points.push_back(getRosPoint(Vector3::Zero()));
  m.colors.push_back(getRosColor(Colors::Blue));
  m.points.push_back(getRosPoint(Vector3::UnitZ() * length));
  m.colors.push_back(getRosColor(Colors::Blue));
  m.pose = getRosPose(pose);
  pub_marker_->publish(m);
}

void VisualizerRos::drawPoints(
    const std::string& ns,
    const size_t id,
    const Positions& points,
    const Color& color,
    const FloatType size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = 0; // add/modify
  FloatType marker_scale = size * viz_scale_;
  m.scale.x = marker_scale;
  m.scale.y = marker_scale;
  m.scale.z = marker_scale;
  m.color = getRosColor(color);
  m.points.reserve(points.cols());
  for(int i = 0; i < points.cols(); ++i)
  {
    if(points.col(i).norm() > 1e4)
    {
      continue;
    }
    m.points.push_back(getRosPoint(points.col(i)));
  }
  pub_marker_->publish(m);
}

void VisualizerRos::drawLines(
    const std::string& ns,
    const size_t id,
    const Lines& lines,
    const Color& color,
    const FloatType size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_;
  m.color = getRosColor(color);
  m.points.reserve(lines.size() * 2);
  for(size_t i = 0; i < lines.size(); ++i)
  {
    m.points.push_back(getRosPoint(lines[i].first));
    m.points.push_back(getRosPoint(lines[i].second));
  }
  pub_marker_->publish(m);
}

void VisualizerRos::drawCoordinateFrames(
    const std::string& ns,
    const size_t id,
    const TransformationVector& poses,
    const FloatType size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_ * 0.05;
  m.colors.reserve(poses.size() * 3);
  m.points.reserve(poses.size() * 3);
  FloatType length = size * viz_scale_;
  for(const Transformation& T : poses)
  {
    const Vector3& p = T.getPosition();
    const Matrix3 R = T.getRotationMatrix();
    m.points.push_back(getRosPoint(p));
    m.colors.push_back(getRosColor(Colors::Red));
    m.points.push_back(getRosPoint(p + R.col(0) * length));
    m.colors.push_back(getRosColor(Colors::Red));
    m.points.push_back(getRosPoint(p));
    m.colors.push_back(getRosColor(Colors::Green));
    m.points.push_back(getRosPoint(p + R.col(1) * length));
    m.colors.push_back(getRosColor(Colors::Green));
    m.points.push_back(getRosPoint(p));
    m.colors.push_back(getRosColor(Colors::Blue));
    m.points.push_back(getRosPoint(p + R.col(2) * length));
    m.colors.push_back(getRosColor(Colors::Blue));
  }
  pub_marker_->publish(m);
}

void VisualizerRos::drawTrajectory(
    const std::string& topic,
    const size_t id,
    const std::vector<Position>& points,
    const Color& color,
    const FloatType size)
{
  if(pub_marker_->getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = topic;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_ * 0.05;
  m.color = getRosColor(color);
  m.points.reserve(points.size());
  for (size_t i = 0u; i < points.size(); ++i)
  {
    m.points.push_back(getRosPoint(points[i]));
  }
  pub_marker_->publish(m);
}

} // namespace ze
