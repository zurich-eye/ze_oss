// Heavily modified by ze.
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

#include <glog/logging.h>
#include <rosbag/view.h>
#include <ze/common/file_utils.h>
#include <ze/common/time_conversions.h>

namespace ze {

RosbagImageQuery::RosbagImageQuery(const std::string& bag_file)
{
  CHECK(fileExists(bag_file)) << "File does not exist: " << bag_file;
  try
  {
    bag_.open(bag_file, rosbag::BagMode::Read);
  }
  catch (rosbag::BagException& exception)
  {
    LOG(FATAL) << "Could not open rosbag: " << bag_file << ": " << exception.what();
  }
}

StampedImage RosbagImageQuery::getStampedImageAtTime(
    const std::string& img_topic,
    const int64_t stamp_ns,
    const FloatType search_range_ms)
{
  // Considering rounding errors.
  const int64_t search_range_ns = millisecToNanosec(search_range_ms);
  ros::Time time_min, time_max;
  time_min.fromNSec(stamp_ns - search_range_ns);
  time_max.fromNSec(stamp_ns + search_range_ns);
  rosbag::View view(
      bag_, rosbag::TopicQuery({img_topic}), time_min, time_max);

  VLOG(100) << "Found messages that fit = " << view.size();
  int64_t best_time_diff = std::numeric_limits<int64_t>::max();
  sensor_msgs::ImageConstPtr best_match_message;
  for (const rosbag::MessageInstance& message : view)
  {
    const int64_t time_diff =
        std::abs(static_cast<int64_t>(message.getTime().toNSec()) - stamp_ns);
    if (time_diff < best_time_diff)
    {
      best_time_diff = time_diff;
      best_match_message = message.instantiate<sensor_msgs::Image>();
      CHECK(best_match_message);
    }
    else
    {
      break; // Already passed relevant time.
    }
  }

  // Extract image
  int64_t best_match_stamp = -1;
  ImageBase::Ptr best_match_img;
  if (best_match_message)
  {
    best_match_stamp = best_match_message->header.stamp.toNSec();
    best_match_img = toImageCpu(*best_match_message);
  }
  else
  {
    LOG(WARNING) << "No image found in bag with this timestamp. If this "
                 << "problem is persistent, you may need to re-index the bag: "
                 << "rosrun ze_rosbag_tools bagrestamper.py -i dataset.bag -o dataset_new.bag";
  }
  return std::make_pair(best_match_stamp, best_match_img);
}

}  // namespace ze
