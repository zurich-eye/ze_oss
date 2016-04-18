#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ze/common/transformation.h>

namespace ze {

Transformation poseMsgTotransformation(
    const geometry_msgs::PoseStamped& pose_msg);

} // ze namespace
