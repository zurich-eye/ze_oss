#pragma once

#include <sensor_msgs/Image.h>
#include <imp/core/image.hpp>

namespace ze {

std::pair<PixelType, PixelOrder> getPixelTypeFromRosImageEncoding(
    const std::string& encoding);

ImageBase::Ptr toImageCpu(
    const sensor_msgs::Image& src,
    PixelOrder pixel_order = PixelOrder::undefined);

ImageBase::Ptr toImageGpu(
    const sensor_msgs::Image& src,
    PixelOrder pixel_order = PixelOrder::undefined);

} // namespace ze
