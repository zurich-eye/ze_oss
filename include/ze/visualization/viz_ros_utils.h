// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <visualization_msgs/Marker.h>
#include <ze/common/types.h>
#include <ze/common/transformation.h>

namespace ze {

inline std_msgs::ColorRGBA getRosColor(const Color& color)
{
  std_msgs::ColorRGBA c;
  c.r = color.r;
  c.g = color.g;
  c.b = color.b;
  c.a = color.a;
  return c;
}

inline geometry_msgs::Point getRosPoint(const Eigen::Ref<const Position>& point)
{
  geometry_msgs::Point p;
  p.x = point(0);
  p.y = point(1);
  p.z = point(2);
  return p;
}

inline geometry_msgs::Quaternion getRosQuaternion(const Quaternion& rot)
{
  geometry_msgs::Quaternion q;
  q.x = rot.toImplementation().x();
  q.y = rot.toImplementation().y();
  q.z = rot.toImplementation().z();
  q.w = rot.toImplementation().w();
  return q;
}

inline geometry_msgs::Pose getRosPose(const Transformation& pose)
{
  geometry_msgs::Pose T;
  T.position = getRosPoint(pose.getPosition());
  T.orientation = getRosQuaternion(pose.getRotation());
  return T;
}

} // namespace ze
