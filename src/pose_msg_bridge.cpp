// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/ros/pose_msg_bridge.hpp>

namespace ze {

Transformation poseMsgTotransformation(
    const geometry_msgs::PoseStamped& pose_msg)
{
  Vector3 p(
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z);
  Quaternion q(
        pose_msg.pose.orientation.w,
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z);
  return Transformation(p, q);
}

geometry_msgs::PoseStamped transformationToPoseStampedMsg(
    const Transformation& T, int64_t stamp)
{
  geometry_msgs::PoseStamped m;
  m.header.stamp = ros::Time().fromNSec(stamp);
  Vector3 p = T.getPosition();
  m.pose.position.x = p.x();
  m.pose.position.y = p.y();
  m.pose.position.z = p.z();
  Quaternion q = T.getRotation();
  m.pose.orientation.x = q.x();
  m.pose.orientation.y = q.y();
  m.pose.orientation.z = q.z();
  m.pose.orientation.w = q.w();
  return m;
}


} // ze namespace
