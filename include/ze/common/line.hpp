#pragma once

#include <ze/common/transformation.h>
#include <ze/common/types.h>

namespace ze {

class Line {
public:

  Line()
    : distance_(0.0) {}

  Line(Quaternion orientation, FloatType distance)
  : orientation_(orientation)
  , distance_(distance) {}

  static void generateLinesFromEndpoints(const Positions& startpoints,
                                         const Positions& endpoints,
                                         std::vector<Line>& lines);

  static FloatType calculateDistanceToLine(const Position& pos,
                                           const Position& line_anchor,
                                           const Vector3& direction)
  {
    return (pos - line_anchor).cross(direction).norm();
  }

  Vector3 getDirection() const
  {
    return orientation_.rotate(Vector3(1.0, 0.0, 0.0));
  }

  Position getAnchorPoint() const
  {
    return distance_ * orientation_.rotate(Vector3(0.0, 0.0, 1.0));
  }

  FloatType calculateDistanceToLine(const Position& pos) const
  {
    return calculateDistanceToLine(pos, getAnchorPoint(), getDirection());
  }

private:
  Quaternion orientation_;
  FloatType distance_;
};

} // namespace ze
