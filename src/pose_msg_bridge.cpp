#include <ze/ros/pose_msg_bridge.h>

namespace ze {

Transformation poseMsgTotransformation(
    const geometry_msgs::PoseStamped& pose_msg)
{
  Eigen::Vector3d p(
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z);
  Eigen::Quaterniond q(
        pose_msg.pose.orientation.w,
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z);
  return Transformation(p, q);
}

} // ze namespace
