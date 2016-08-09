#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ze/common/transformation.h>

namespace ze {

Transformation poseMsgTotransformation(
    const geometry_msgs::PoseStamped& pose_msg);

geometry_msgs::PoseStamped transformationToPoseMsg(
    const Transformation& T, int64_t stamp);


} // ze namespace
