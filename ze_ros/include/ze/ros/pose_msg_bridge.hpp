// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ze/common/transformation.hpp>

namespace ze {

Transformation poseMsgTotransformation(
    const geometry_msgs::PoseStamped& pose_msg);

geometry_msgs::PoseStamped transformationToPoseStampedMsg(
    const Transformation& T, int64_t stamp);


} // ze namespace
