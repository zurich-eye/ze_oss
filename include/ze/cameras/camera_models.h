#pragma once

#include <cmath>

namespace ze {
namespace internal {

// Pinhole projection model.
// Pure static, intended to be used in both GPU and CPU programming.
struct PinholeProjection
{
  template <typename T>
  static void project(const T* bearing, const T* params, T* px)
  {
    const T fx = params[0];
    const T fy = params[1];
    const T cx = params[2];
    const T cy = params[3];
    const T u = bearing[0] / bearing[2];
    const T v = bearing[1] / bearing[2];
    px[0] = u * fx + cx;
    px[1] = v * fy + cy;
  }

  template <typename T>
  static void backProject(const T* px, const T* params, T* bearing)
  {
    const T fx = params[0];
    const T fy = params[1];
    const T cx = params[2];
    const T cy = params[3];
    bearing[0] = (px[0] - cx) / fx;
    bearing[1] = (px[1] - cy) / fy;
    bearing[2] = 1.0;
  }

  template <typename T>
  static void dProject_dBearing(const T* bearing, const T* params, T* H)
  {
    const T fx = params[0];
    const T fy = params[1];
    const T z_sq = bearing[2] * bearing[2];
    const T z_inv = 1.0 / bearing[2];
    H[0] = fx * z_inv;
    H[1] = 0;
    H[2] = 0;
    H[3] = fy * z_inv;
    H[4] = - fx * bearing[0] / z_sq;
    H[5] = - fy * bearing[1] / z_sq;
  }
};

// This class implements the FOV distortion model of Deverneay and Faugeras,
// Straight lines have to be straight, 2001. In PTAM this model is called ATAN.
struct FovDistortion
{
  template <typename T>
  static void distort(const T* params, T* px)
  {
    const T s = params[0];
    const T rad = std::sqrt(px[0] * px[0] + px[1] * px[1]);
    const T factor = (rad < 0.001) ? 1.0 : std::atan(rad * 2.0 * std::tan(s / 2.0)) / s;
    px[0] *= factor;
    px[1] *= factor;
  }

  template <typename T>
  static void undistort(const T* params, T* px)
  {
    const T s = params[0];
    const T rad = std::sqrt(px[0] * px[0] + px[1] * px[1]);
    const T factor = (rad < 0.001) ? 1.0 : (std::tan(rad * s) / (2.0 * std::tan(s / 2.0))) / rad;
    px[0] *= factor;
    px[1] *= factor;
  }
};

// This class implements the radial and tangential distortion model used by
// OpenCV and ROS. Reference:
// docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
struct RadialTangentialDistortion
{
  template <typename T>
  static void distort(const T* params, T* px)
  {
    const T k1 = params[0];
    const T k2 = params[1];
    const T p1 = params[2];
    const T p2 = params[3];
    const T xx = px[0] * px[0];
    const T yy = px[1] * px[1];
    const T xy2 = 2.0 * px[0] * px[1];
    const T r2 = xx + yy;
    const T cdist = (k1 + k2 * r2) * r2;
    px[0] += px[0] * cdist + p1 * xy2 + p2 * (r2 + 2.0 * xx);
    px[1] += px[1] * cdist + p2 * xy2 + p1 * (r2 + 2.0 * yy);
  }

  template <typename T>
  static void undistort(const T* params, T* px)
  {
    const T k1 = params[0];
    const T k2 = params[1];
    const T p1 = params[2];
    const T p2 = params[3];
    T x0 = px[0], y0 = px[1];
    for(int i = 0; i < 5; ++i)
    {
      const T xx = px[0] * px[0];
      const T yy = px[1] * px[1];
      const T xy2 = 2.0 * px[0] * px[1];
      const T r2 = xx + yy;
      const T icdist = 1.0 / (1.0 + (k1 + k2 * r2) * r2);
      const T dx = p1 * xy2 + p2 * (r2 + 2.0 * xx);
      const T dy = p2 * xy2 + p1 * (r2 + 2.0 * yy);
      px[0] = (x0 - dx) * icdist;
      px[1] = (y0 - dy) * icdist;
    }
  }
};

// This class implements the distortion model described in the paper:
// "A Generic Camera Model and Calibration Method for Conventional, Wide-Angle,
// and Fish-Eye Lenses" by Juho Kannala and Sami S. Brandt, PAMI.
struct EquidistantDistortion
{
  template <typename T>
  static void distort(const T* params, T* px)
  {
    const T k1 = params[0];
    const T k2 = params[1];
    const T k3 = params[2];
    const T k4 = params[3];
    const T r = std::sqrt(px[0] * px[0] + px[1] * px[1]);
    const T theta = std::atan(r);
    const T theta2 = theta * theta;
    const T theta4 = theta2 * theta2;
    const T theta6 = theta4 * theta2;
    const T theta8 = theta4 * theta4;
    const T thetad = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    const T scaling = (r > 1e-8) ? thetad / r : 1.0;
    px[0] *= scaling;
    px[1] *= scaling;
  }

  template <typename T>
  static void undistort(const T* params, T* px)
  {
    const T k1 = params[0];
    const T k2 = params[1];
    const T k3 = params[2];
    const T k4 = params[3];
    const T thetad = std::sqrt(px[0] * px[0] + px[1] * px[1]);
    T theta = thetad;
    for(int i = 0; i < 5; ++i)
    {
      const T theta2 = theta * theta;
      const T theta4 = theta2 * theta2;
      const T theta6 = theta4 * theta2;
      const T theta8 = theta4 * theta4;
      theta = thetad / (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    }
    const T scaling = std::tan(theta) / thetad;
    px[0] *= scaling;
    px[1] *= scaling;
  }
};

} // namespace internal
} // namespace ze
