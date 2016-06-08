#include <ze/common/line.hpp>

namespace ze {

Matrix26 dLineMeasurement_dPose(const Transformation& T_B_W,
                                const Transformation& T_C_B,
                                const LineMeasurement& measurement,
                                const Position& line_anchor,
                                const Vector3& line_direction)
{
  //! @todo Can be optimized.
  const Transformation T_C_W = T_C_B * T_B_W;
  const Matrix13 measurement_W_transpose =
      T_C_W.getRotation().inverse().rotate(measurement).transpose();
  const Vector3 cam_to_anchor = T_C_W.inverse().getPosition() - line_anchor;
  const FloatType inverse_distance = 1.0 / cam_to_anchor.norm();
  const Vector3 cam_to_anchor_normalized = cam_to_anchor * inverse_distance;
  const Matrix3 d_cam_to_anchor_normalized_d_campos =
      inverse_distance * (Matrix3::Identity() -
                          cam_to_anchor_normalized *
                          cam_to_anchor_normalized.transpose());

  Matrix26 J;
  J.block<1, 3>(0, 0).setZero();
  J.block<1, 3>(0, 3) =
      -measurement_W_transpose * skewSymmetric(line_direction);

  J.block<1, 3>(1, 0) =
      -measurement_W_transpose * d_cam_to_anchor_normalized_d_campos;
  J.block<1, 3>(1, 3) =
      -measurement_W_transpose * (
        d_cam_to_anchor_normalized_d_campos *
        skewSymmetric(T_C_W.getRotation().inverse().rotate(T_C_B.getPosition()) +
         T_B_W.getRotation().inverse().rotate(T_B_W.getPosition()))
        + skewSymmetric(cam_to_anchor_normalized));
  return J;
}

void generateRandomLines(size_t num_lines, Lines& lines,
                         Positions* startpoints,
                         Positions* endpoints)
{
  ze::Positions start, end;
  start.setRandom(3, num_lines);
  end.setRandom(3, num_lines);
  ze::generateLinesFromEndpoints(start, end, lines);
  if (startpoints != nullptr && endpoints != nullptr)
  {
    *startpoints = start;
    *endpoints = end;
  }
}

void generateLinesFromEndpoints(const Positions& startpoints,
                                const Positions& endpoints,
                                Lines& lines)
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
