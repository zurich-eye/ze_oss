// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <tf/tf.h>
#include <ze/common/transformation.hpp>

namespace ze {

tf::Transform transformationToTF(const Transformation& ze_T);

} // ze namespace
