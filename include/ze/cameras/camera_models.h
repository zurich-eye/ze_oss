#pragma once

#include <iostream>
#include <cmath>

namespace ze {

// Pure static camera projection and distortion models, intended to be used in
// both GPU and CPU code. Parameter checking should be performed in interface
// classes.

// Pinhole projection model.
struct PinholeGeometry
{
  template <typename T>
  static void project(const T* params, T* px)
  {
    const T fx = params[0];
    const T fy = params[1];
    const T cx = params[2];
    const T cy = params[3];
    px[0] = px[0] * fx + cx;
    px[1] = px[1] * fy + cy;
  }

  template <typename T>
  static void backProject(const T* params, T* px)
  {
    const T fx = params[0];
    const T fy = params[1];
    const T cx = params[2];
    const T cy = params[3];
    px[0] = (px[0] - cx) / fx;
    px[1] = (px[1] - cy) / fy;
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




enum class DistortionType
{
  No,
  Fov,
  RadTan,
  Equidistant,
};

// -----------------------------------------------------------------------------
// Dummy distortion.
struct NoDistortion
{
  static constexpr DistortionType type = DistortionType::No;

  template <typename T>
  static void distort(const T* params, T* px)
  {}

  template <typename T>
  static void undistort(const T* params, T* px)
  {}

  template <typename T>
  static void dDistort_dPx(const T* params, const T* /*px_unitplane*/, T* jac_colmajor)
  {
    T& J_00 = jac_colmajor[0];
    T& J_10 = jac_colmajor[1];
    T& J_01 = jac_colmajor[2];
    T& J_11 = jac_colmajor[3];
    J_00 = 1.0;
    J_01 = 0.0;
    J_10 = 0.0;
    J_11 = 1.0;
  }
};

// -----------------------------------------------------------------------------
// This class implements the FOV distortion model of Deverneay and Faugeras,
// Straight lines have to be straight, 2001. In PTAM this model is called ATAN.
struct FovDistortion
{
  static constexpr DistortionType type = DistortionType::Fov;

  template <typename T>
  static void distort(const T* params, T* px)
  {
    const T s = params[0];
    const T rad = std::sqrt(px[0] * px[0] + px[1] * px[1]);
    const T factor = (rad < 0.001) ? 1.0 : std::atan(rad * 2.0 * std::tan(s / 2.0)) / (s * rad);
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

  template <typename T>
  static void dDistort_dPx(const T* params, const T* px_unitplane, T* jac_colmajor)
  {
    const T x = px_unitplane[0];
    const T y = px_unitplane[1];
    const T s = params[0];
    const T xx = x * x;
    const T yy = y * y;
    const T rad_sq = xx + yy;
    T& J_00 = jac_colmajor[0];
    T& J_10 = jac_colmajor[1];
    T& J_01 = jac_colmajor[2];
    T& J_11 = jac_colmajor[3];
    if(s * s < 1e-5)
    {
      // Distortion parameter very small.
      J_00 = 1.0; J_01 = 0.0;
      J_10 = 0.0; J_11 = 1.0;
    }
    else if(rad_sq < 1e-5) // Projection very close to image center
    {
      J_00 = 2.0 * std::tan(s / 2.0) / s;
      J_11 = J_00;
      J_01 = 0.0;
      J_10 = 0.0;
    }
    else // Standard case
    {
      const T xy = x * y;
      const T rad = std::sqrt(rad_sq);
      const T tan_s_half_x2 = std::tan(s / 2.0) * 2.0;
      const T nominator = std::atan(tan_s_half_x2 * rad);
      const T offset = nominator / (s * rad);
      const T scale =
          tan_s_half_x2 / (s * (xx + yy) * (tan_s_half_x2 * tan_s_half_x2 * (xx + yy) + 1.0))
          - nominator / (s * rad * rad_sq);
      J_00 = xx * scale + offset;
      J_11 = yy * scale + offset;
      J_01 = xy * scale;
      J_10 = J_01;
    }   
  }
};

// -----------------------------------------------------------------------------
// This class implements the radial and tangential distortion model used by
// OpenCV and ROS. Reference:
// docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
struct RadialTangentialDistortion
{
  static constexpr DistortionType type = DistortionType::RadTan;

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

  template <typename T>
  static void dDistort_dPx(const T* params, const T* px_unitplane, T* jac_colmajor)
  {
    const T k1 = params[0];
    const T k2 = params[1];
    const T p1 = params[2];
    const T p2 = params[3];
    const T x = px_unitplane[0];
    const T y = px_unitplane[1];
    const T xx = x * x;
    const T yy = y * y;
    const T xy = x * y;
    const T r2 = xx + yy;
    const T k2_r2_x4 = k2 * r2 * 4.0;
    const T cdist_p1 = (k1 + k2 * r2) * r2 + 1.0;
    T& J_00 = jac_colmajor[0];
    T& J_10 = jac_colmajor[1];
    T& J_01 = jac_colmajor[2];
    T& J_11 = jac_colmajor[3];
    J_00 = cdist_p1 + k1 * 2.0 * xx + k2_r2_x4 * xx + 2.0 * p1 * y + 6.0 * p2 * x;
    J_11 = cdist_p1 + k1 * 2.0 * yy + k2_r2_x4 * yy + 2.0 * p2 * x + 6.0 * p1 * y;
    J_10 = 2.0 * k1 * xy + k2_r2_x4 * xy + 2.0 * p1 * x + 2.0 * p2 * y;
    J_01 = J_10;
  }
};

// -----------------------------------------------------------------------------
// This class implements the distortion model described in the paper:
// "A Generic Camera Model and Calibration Method for Conventional, Wide-Angle,
// and Fish-Eye Lenses" by Juho Kannala and Sami S. Brandt, PAMI.
struct EquidistantDistortion
{
  static constexpr DistortionType type = DistortionType::Equidistant;

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

  template <typename T>
  static void dDistort_dPx(const T* params, const T* px_unitplane, T* jac_colmajor)
  {
    const T k1 = params[0];
    const T k2 = params[1];
    const T k3 = params[2];
    const T k4 = params[3];
    const T x = px_unitplane[0];
    const T y = px_unitplane[1];
    const T r = std::sqrt(x * x + y * y);

    T& J_00 = jac_colmajor[0];
    T& J_10 = jac_colmajor[1];
    T& J_01 = jac_colmajor[2];
    T& J_11 = jac_colmajor[3];

    if(r < 1e-7)
    {
      J_00 = 1.0; J_01 = 0.0;
      J_10 = 0.0; J_11 = 1.0;
    }
    else
    {
      T xx = x * x;
      T yy = y * y;
      T xy = x * y;
      T rad_sq = xx + yy;
      T rad = std::sqrt(rad_sq);
      T atan_rad = std::atan(rad);
      T atan_rad_inv_rad = atan_rad / rad;
      T atan_rad_sq = atan_rad * atan_rad;
      T atan_rad_cubic = atan_rad_sq * atan_rad_sq;

      T t1 = 1.0 / (xx + yy + 1.0);
      T t2 = k1 * atan_rad_sq
           + k2 * atan_rad_cubic
           + k3 * atan_rad_cubic * atan_rad_sq
           + k4 * (atan_rad_cubic * atan_rad_cubic) + 1.0;
      T t3 = t1 * atan_rad_inv_rad;

      T offset = t2 * atan_rad_inv_rad;
      T scale  = t2 * (t1 / rad_sq - atan_rad_inv_rad / rad_sq)
          + atan_rad_inv_rad * t3 * (
                2.0 * k1
              + 4.0 * k2 * atan_rad_sq
              + 6.0 * k3 * atan_rad_cubic
              + 8.0 * k4 * atan_rad_cubic * atan_rad_sq);

      J_11 = yy * scale + offset;
      J_00 = xx * scale + offset;
      J_01 = xy * scale;
      J_10 = J_01;
    }
  }
};

} // namespace ze
