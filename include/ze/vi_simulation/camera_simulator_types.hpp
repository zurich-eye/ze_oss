// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/types.h>

namespace ze {

struct CameraMeasurements
{
  //! Each column is a keypoint observation.
  Keypoints keypoints_;

  //! Global landmark index of each observed feature. The size of the vector is
  //! the same as the number of columns in the keypoints block.
  std::vector<int32_t> global_landmark_ids_;

  //! Temporary track index of a landmark. If the landmark is
  //! re-observed after a loop, it will be assigned a different id.
  std::vector<int32_t> local_track_ids_;
};
using CameraMeasurementsVector = std::vector<CameraMeasurements>;

} // namespace ze
