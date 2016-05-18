// Modified by ze.
// Copyright (c) 2016, Robotics and Perception Group, Titus Cieslewski
// All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation and
// or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.

#include <ze/ros/rosbag_image_query.hpp>

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <rosbag/view.h>

namespace ze {

RosbagImageQuery::RosbagImageQuery(const std::string& bag_file)
{
  try
  {
    bag_.open(bag_file, rosbag::BagMode::Read);
  }
  catch (rosbag::BagException& exception)
  {
    LOG(FATAL) << "Opening bag failed: " << exception.what();
  }
}

ImageBase::Ptr RosbagImageQuery::getImageAtTime(
    const std::string& img_topic, const int64_t stamp_ns);
{
  // Considering rounding errors.
  constexpr int64_t search_range_ns = 10000;
  ros::Time time, time_min, time_max;
  time.fromNSec(stamp_ns);
  time_min.fromNSec(stamp_ns - search_range_ns);
  time_max.fromNSec(stamp_ns + search_range_ns);
  rosbag::View view(
      bag_, rosbag::TopicQuery({img_topic}), time_min, time_max);

  int64_t best_time_diff = std::numeric_limits<int64_t>::max();
  for (const rosbag::MessageInstance& message : view)
  {
    const int64_t time_distance_s = std::abs((message.getTime() - time).toNSec());
    if (time_distance_s < min_time_distance_s)
    {
      min_time_distance_s = time_distance_s;
      const sensor_msgs::ImageConstPtr image_message =
          message.instantiate<sensor_msgs::Image>();
      CHECK(image_message);
      *result = cv_bridge::toCvCopy(image_message)->image;
    }
    else
    {
      // Already passed relevant time.
      break;
    }
  }

  return min_time_distance_s != std::numeric_limits<double>::max();
}

}  // namespace ze
