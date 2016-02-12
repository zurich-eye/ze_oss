#pragma once

#include <ze/common/types.h>
#include <ze/visualization/viz_common.h>

// fwd
namespace gtsam {
class Values;
}

namespace ze {

// fwd
class Visualizer;

void drawGtsamPoint3(
    Visualizer& visualizer,
    const gtsam::Values& values,
    const char key_prefix,
    const std::string& ns,
    const size_t id,
    const Color& color,
    const FloatType size = 0.02);

void drawGtsamPose3(
    Visualizer& visualizer,
    const gtsam::Values& values,
    const char key_prefix,
    const std::string& ns,
    const size_t id,
    const FloatType size = 0.02);

} // namespace ze
