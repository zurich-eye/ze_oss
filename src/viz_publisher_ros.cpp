#include <ze/visualization/viz_publisher_ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ze {

namespace utils {

std_msgs::ColorRGBA getRosColor(const Color& color)
{
  std_msgs::ColorRGBA c;
  c.r = color.r;
  c.g = color.g;
  c.b = color.b;
  c.a = color.a;
  return c;
}

geometry_msgs::Point getRosPoint(const Eigen::Ref<const Position>& point)
{
  geometry_msgs::Point p;
  p.x = point(0);
  p.y = point(1);
  p.z = point(2);
  return p;
}

geometry_msgs::Quaternion getRosQuaternion(const Quaternion& rot)
{
  geometry_msgs::Quaternion q;
  q.x = rot.toImplementation().x();
  q.y = rot.toImplementation().y();
  q.z = rot.toImplementation().z();
  q.w = rot.toImplementation().w();
  return q;
}

geometry_msgs::Pose getRosPose(const Transformation& pose)
{
  geometry_msgs::Pose T;
  T.position = getRosPoint(pose.getPosition());
  T.orientation = getRosQuaternion(pose.getRotation());
  return T;
}

} // namespace utils

VisualizerRos::VisualizerRos(
    const ros::NodeHandle& nh_private,
    const std::string& marker_topic)
  : pnh_(nh_private)
  , pub_marker_(pnh_.advertise<visualization_msgs::Marker>(marker_topic, 100))
{}

void VisualizerRos::drawPoint(
    const std::string& ns,
    const size_t id,
    const Position& point,
    const Color& color,
    const FloatType size)
{
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
  m.color = utils::getRosColor(color);
  m.pose.position = utils::getRosPoint(point);
  pub_marker_.publish(m);
}

void VisualizerRos::drawLine(
    const std::string& ns,
    const size_t id,
    const Position& line_from,
    const Position& line_to,
    const Color& color,
    const FloatType size)
{
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
  m.color = utils::getRosColor(color);
  m.points.reserve(2);
  m.points.push_back(utils::getRosPoint(line_from));
  m.points.push_back(utils::getRosPoint(line_to));
  pub_marker_.publish(m);
}

void VisualizerRos::drawCoordinateFrame(
    const std::string& ns,
    const size_t id,
    const Transformation& pose, // T_W_B
    const FloatType size)
{
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
  m.points.push_back(utils::getRosPoint(p));
  m.colors.push_back(utils::getRosColor(Colors::Red));
  m.points.push_back(utils::getRosPoint(p + Vector3::UnitX() * length));
  m.colors.push_back(utils::getRosColor(Colors::Red));
  m.points.push_back(utils::getRosPoint(p));
  m.colors.push_back(utils::getRosColor(Colors::Green));
  m.points.push_back(utils::getRosPoint(p + Vector3::UnitY() * length));
  m.colors.push_back(utils::getRosColor(Colors::Green));
  m.points.push_back(utils::getRosPoint(p));
  m.colors.push_back(utils::getRosColor(Colors::Blue));
  m.points.push_back(utils::getRosPoint(p + Vector3::UnitZ() * length));
  m.colors.push_back(utils::getRosColor(Colors::Blue));
  m.pose = utils::getRosPose(pose);
  pub_marker_.publish(m);
}

void VisualizerRos::drawPoints(
    const std::string& ns,
    const size_t id,
    const Positions& points,
    const Color& color,
    const FloatType size)
{
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
  m.color = utils::getRosColor(color);
  m.points.reserve(points.cols());
  for(int i = 0; i < points.cols(); ++i)
  {
    m.points.push_back(utils::getRosPoint(points.col(i)));
  }
  pub_marker_.publish(m);
}

void VisualizerRos::drawLines(
    const std::string& ns,
    const size_t id,
    const Lines& lines,
    const Color& color,
    const FloatType size)
{
  visualization_msgs::Marker m;
  m.header.frame_id = world_frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = 0; // 0 = add/modify
  m.scale.x = size * viz_scale_;
  m.color = utils::getRosColor(color);
  m.points.reserve(lines.size() * 2);
  for(size_t i = 0; i < lines.size(); ++i)
  {
    m.points.push_back(utils::getRosPoint(lines[i].first));
    m.points.push_back(utils::getRosPoint(lines[i].second));
  }
  pub_marker_.publish(m);
}

void VisualizerRos::drawCoordinateFrames(
    const std::string& ns,
    const size_t id,
    const TransformationVector& poses,
    const FloatType size)
{
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
    m.points.push_back(utils::getRosPoint(p));
    m.colors.push_back(utils::getRosColor(Colors::Red));
    m.points.push_back(utils::getRosPoint(p + R.col(0) * length));
    m.colors.push_back(utils::getRosColor(Colors::Red));
    m.points.push_back(utils::getRosPoint(p));
    m.colors.push_back(utils::getRosColor(Colors::Green));
    m.points.push_back(utils::getRosPoint(p + R.col(1) * length));
    m.colors.push_back(utils::getRosColor(Colors::Green));
    m.points.push_back(utils::getRosPoint(p));
    m.colors.push_back(utils::getRosColor(Colors::Blue));
    m.points.push_back(utils::getRosPoint(p + R.col(2) * length));
    m.colors.push_back(utils::getRosColor(Colors::Blue));
  }
  pub_marker_.publish(m);
}

} // namespace ze
