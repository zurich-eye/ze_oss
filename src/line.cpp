#include <ze/common/line.hpp>

namespace ze {

void Line::generateLinesFromEndpoints(const Positions& startpoints,
                                             const Positions& endpoints,
                                             std::vector<Line>& lines)
{
  CHECK_EQ(startpoints.cols(), endpoints.cols());
  const size_t n = startpoints.cols();
  lines.clear();
  lines.reserve(n);
  for (size_t i = 0; i < n; ++i)
  {
    const Vector3& start = startpoints.col(i);
    const Vector3& end = endpoints.col(i);
    const Vector3 direction = (end - start).normalized();
    const double anchor_point_distance = (start.squaredNorm() - start.dot(end)) /
                                         (end - start).norm();
    const Vector3 anchor_point = start + anchor_point_distance * direction;
    const Vector3 anchor_point_direction = anchor_point.normalized();
    Matrix3 rotation;
    rotation << direction, anchor_point_direction.cross(direction), anchor_point_direction;
    const Quaternion orientation(rotation);
    lines.emplace_back(orientation, anchor_point.norm());
  }
}

} // namespace ze
