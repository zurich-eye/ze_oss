#pragma once

#include <ze/common/transformation.h>
#include <ze/common/types.h>

namespace ze {

class Line;

// Convenience typedefs:
using Lines = std::vector<Line>;

inline FloatType calculateDistanceToLine(const Position& pos,
                                         const Position& line_anchor,
                                         const Vector3& direction)
{
  return (pos - line_anchor).cross(direction).norm();
}

Matrix26 dLineMeasurement_dPose(const Transformation& T_B_W,
                                const Transformation& T_C_B,
                                const LineMeasurement& measurement,
                                const Position& line_anchor,
                                const Vector3& line_direction);


void generateRandomLines(size_t num_lines, Lines& lines,
                         Positions* startpoints = nullptr,
                         Positions* endpoints = nullptr);

void generateLinesFromEndpoints(const Positions& startpoints,
                                const Positions& endpoints,
                                Lines& lines);

class Line
{
public:

  Line() = default;

  Line(Quaternion orientation, FloatType distance)
  : orientation_(orientation)
  , distance_(distance) {}


  Vector3 direction() const
  {
    return orientation_.rotate(Vector3(1.0, 0.0, 0.0));
  }

  Position anchorPoint() const
  {
    return distance_ * orientation_.rotate(Vector3(0.0, 0.0, 1.0));
  }

  FloatType calculateDistanceToLine(const Position& pos) const
  {
    return ze::calculateDistanceToLine(pos, anchorPoint(), direction());
  }

  Vector2 calculateMeasurementError(const Vector3& measurement_W,
                                    const Vector3& camera_position_W) const;

private:
  Quaternion orientation_;
  FloatType distance_ = 0.0;
};

} // namespace ze
