#include <ze/geometry/line.hpp>

#include <ze/cameras/camera_utils.h>

namespace ze {

// -----------------------------------------------------------------------------
Vector2 Line::calculateMeasurementError(const Vector3& measurement_W,
                                        const Vector3& camera_position_W) const
{
  const Vector3 anchor_to_camera = camera_position_W - anchorPoint();
  CHECK_GT(anchor_to_camera.norm(), 0.0);
  return Vector2(measurement_W.dot(direction()),
                 measurement_W.dot(anchor_to_camera) / anchor_to_camera.norm());
}

// -----------------------------------------------------------------------------
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
  const Vector3 anchor_to_cam = T_C_W.inverse().getPosition() - line_anchor;
  CHECK_NE(anchor_to_cam.norm(), 0.0);
  const FloatType inverse_distance = 1.0 / anchor_to_cam.norm();
  const Vector3 anchor_to_cam_normalized = anchor_to_cam * inverse_distance;
  const Matrix3 d_anchor_to_cam_normalized_d_campos =
      inverse_distance * (Matrix3::Identity() -
                          anchor_to_cam_normalized *
                          anchor_to_cam_normalized.transpose());

  Matrix26 J;
  J.block<1, 3>(0, 0).setZero();
  J.block<1, 3>(0, 3) =
      -measurement_W_transpose * skewSymmetric(line_direction);

  J.block<1, 3>(1, 0) =
      -measurement_W_transpose * d_anchor_to_cam_normalized_d_campos;
  J.block<1, 3>(1, 3) =
      -measurement_W_transpose * (
        d_anchor_to_cam_normalized_d_campos *
        skewSymmetric(T_C_W.getRotation().inverse().rotate(T_C_B.getPosition()) +
         T_B_W.getRotation().inverse().rotate(T_B_W.getPosition()))
        + skewSymmetric(anchor_to_cam_normalized));

  return J;
}

// -----------------------------------------------------------------------------
void generateRandomVisibleLines(const Camera& cam, const Transformation& T_W_C,
                                size_t num_lines, Lines& lines_W,
                                Positions* startpoints_W, Positions* endpoints_W)
{
  auto visible_points_C =
      generateRandomVisible3dPoints(cam, num_lines * 2, 10, 1.0, 20.0);

  Positions start_W =
      T_W_C.transformVectorized(
        std::get<2>(visible_points_C).topLeftCorner(3, num_lines));
  Positions end_W =
      T_W_C.transformVectorized(
        std::get<2>(visible_points_C).topRightCorner(3, num_lines));

  generateLinesFromEndpoints(start_W, end_W, lines_W);

  if (startpoints_W != nullptr && endpoints_W != nullptr)
  {
    *startpoints_W = start_W;
    *endpoints_W = end_W;
  }
}

// -----------------------------------------------------------------------------
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
    Vector3 direction = (end - start).normalized();
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
