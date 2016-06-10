// Heavily modified by ze.
// RPG-BSD License, Titus Cieslewski
#pragma once

#include <rosbag/bag.h>
#include <string>
#include <vector>

#include <imp/bridge/ros/ros_bridge.hpp>
#include <imp/core/image.hpp>
#include <ze/common/macros.h>

namespace ze {

class RosbagImageQuery
{
public:
  ZE_POINTER_TYPEDEFS(RosbagImageQuery);

  RosbagImageQuery() = default;
  RosbagImageQuery(const std::string& bagfile_path);
  ~RosbagImageQuery() = default;

  bool loadRosbag(const std::string& bagfile_path);

  StampedImage getStampedImageAtTime(
      const std::string& img_topic,
      const int64_t stamp_ns,
      const FloatType search_range_ms = 10.0);

private:
  rosbag::Bag bag_;
};

}  // namespace ze
