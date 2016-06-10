#ifndef IMP_CU_PINHOLE_CAMERA_CUH
#define IMP_CU_PINHOLE_CAMERA_CUH

#include <cuda_runtime_api.h>
#include <imp/core/pixel.hpp>
#include <imp/cu_core/cu_matrix.cuh>

namespace ze {
namespace cu {

// Pinhole projection model.
struct PinholeGeometry
{
  template <typename T>
  __device__
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
  __device__
  static void backProject(const T* params, T* px)
  {
    const T fx = params[0];
    const T fy = params[1];
    const T cx = params[2];
    const T cy = params[3];
    px[0] = (px[0] - cx) / fx;
    px[1] = (px[1] - cy) / fy;
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
// This class implements the radial and tangential distortion model used by
// OpenCV and ROS. Reference:
// docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
struct RadialTangentialDistortion
{
  static constexpr DistortionType type = DistortionType::RadTan;

  template <typename T>
  __device__
  static void distort(const T* params, T* px, T* jac_colmajor = nullptr)
  {
    const T x = px[0];
    const T y = px[1];
    const T k1 = params[0];
    const T k2 = params[1];
    const T p1 = params[2];
    const T p2 = params[3];
    const T xx = x * x;
    const T yy = y * y;
    const T xy = x * y;
    const T r2 = xx + yy;
    const T cdist = (k1 + k2 * r2) * r2;
    px[0] += px[0] * cdist + p1 * 2.0 * xy + p2 * (r2 + 2.0 * xx);
    px[1] += px[1] * cdist + p2 * 2.0 * xy + p1 * (r2 + 2.0 * yy);

    if (jac_colmajor)
    {
      const T k2_r2_x4 = k2 * r2 * 4.0;
      const T cdist_p1 = cdist + 1.0;
      T& J_00 = jac_colmajor[0];
      T& J_10 = jac_colmajor[1];
      T& J_01 = jac_colmajor[2];
      T& J_11 = jac_colmajor[3];
      J_00 = cdist_p1 + k1 * 2.0 * xx + k2_r2_x4 * xx + 2.0 * p1 * y + 6.0 * p2 * x;
      J_11 = cdist_p1 + k1 * 2.0 * yy + k2_r2_x4 * yy + 2.0 * p2 * x + 6.0 * p1 * y;
      J_10 = 2.0 * k1 * xy + k2_r2_x4 * xy + 2.0 * p1 * x + 2.0 * p2 * y;
      J_01 = J_10;
    }
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
  __device__
  static void distort(const T* params, T* px, T* jac_colmajor = nullptr)
  {
    const T x = px[0];
    const T y = px[1];
    const T k1 = params[0];
    const T k2 = params[1];
    const T k3 = params[2];
    const T k4 = params[3];
    const T r_sqr = x * x + y * y;
    const T r = std::sqrt(r_sqr);
    const T theta = std::atan(r);
    const T theta2 = theta * theta;
    const T theta4 = theta2 * theta2;
    const T theta6 = theta4 * theta2;
    const T theta8 = theta4 * theta4;
    const T thetad = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    const T scaling = (r > 1e-8) ? thetad / r : 1.0;
    px[0] *= scaling;
    px[1] *= scaling;

    if (jac_colmajor)
    {
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
        T theta_inv_r = theta / r;
        T theta_sqr = theta * theta;
        T theta_four = theta_sqr * theta_sqr;

        T t1 = 1.0 / (xx + yy + 1.0);
        T t2 = k1 * theta_sqr
            + k2 * theta_four
            + k3 * theta_four * theta_sqr
            + k4 * (theta_four * theta_four) + 1.0;
        T t3 = t1 * theta_inv_r;

        T offset = t2 * theta_inv_r;
        T scale  = t2 * (t1 / r_sqr - theta_inv_r / r_sqr)
            + theta_inv_r * t3 * (
              2.0 * k1
              + 4.0 * k2 * theta_sqr
              + 6.0 * k3 * theta_four
              + 8.0 * k4 * theta_four * theta_sqr);

        J_11 = yy * scale + offset;
        J_00 = xx * scale + offset;
        J_01 = xy * scale;
        J_10 = J_01;
      }
    }
  }
};

/**
 * @brief The PinholeCamera class implements a very simple pinhole camera model.
 *        Points in the image plane are denoted by the coordinate vectors (u,v) whereas
 *        world coordinates use the (x,y,z) nominclatur.
 *
 * @todo (MWE) marry off with other camera implementations (maybe with aslam_cv2 camera models? -> resolve Eigen vs. CUDA issues first)
 *
 */
class PinholeCamera
{
public:
  typedef std::shared_ptr<PinholeCamera> Ptr;

  __host__ PinholeCamera() = default;
  __host__ ~PinholeCamera() = default;

  __host__ PinholeCamera(float fu, float fv, float cu, float cv)
    : f_(fu, fv)
    , c_(cu, cv)
  {
  }

  // copy and asignment operator
  __host__ __device__
  PinholeCamera(const PinholeCamera& other)
    : f_(other.f())
    , c_(other.c())
  {
  }
  __host__ __device__
  PinholeCamera& operator=(const PinholeCamera& other)
  {
    if  (this != &other)
    {
      f_ = other.f();
      c_ = other.c();
    }
    return *this;
  }

  __host__ __device__ __forceinline__
  ze::cu::Matrix3f intrinsics()
  {
    ze::cu::Matrix3f K;
    K(0,0) = f_.x;
    K(0,1) = 0.0f;
    K(0,2) = c_.x;
    K(1,0) = 0.0f;
    K(1,1) = f_.y;
    K(1,2) = c_.y;
    K(2,0) = 0.0f;
    K(2,1) = 0.0f;
    K(2,2) = 1.0f;
    return K;
  }

  /** Multiplies the focal length as well as the principal point with the given \a scale \a factor
   */
  __host__ __device__ __forceinline__
  void scale(float scale_factor)
  {
    f_ *= scale_factor;
    c_ *= scale_factor;
  }

  __host__ __device__ __forceinline__
  PinholeCamera& operator*=(float rhs)
  {
    this->scale(rhs);
    return *this;
  }
  __host__ __device__ __forceinline__
  PinholeCamera operator*(float rhs) const
  {
    PinholeCamera result = *this;
    result *= rhs;
    return result;
  }


  //  __host__ __device__ __forceinline__
  //  Vec32fC3 cam2world(const Vec32fC2& uv) const
  //  {
  //    return Vec32fC3((uv.x-c_.x)/f_.x,
  //                    (uv.y-c_.y)/f_.y,
  //                    1.0f);
  //  }

  //  __host__ __device__ __forceinline__
  //  Vec32fC2 world2cam(const Vec32fC3& p) const
  //  {
  //    return Vec32fC2(f_.x*p.x/p.z + c_.x,
  //                    f_.y*p.y/p.z + c_.y);
  //  }

  __host__ __device__ __forceinline__
  float3 cam2world(const float2& uv) const
  {
    return make_float3((uv.x-c_.x)/f_.x,
                       (uv.y-c_.y)/f_.y,
                       1.0f);
  }

  __host__ __device__ __forceinline__
  float2 world2cam(const float3& p) const
  {
    return make_float2(f_.x*p.x/p.z + c_.x,
                       f_.y*p.y/p.z + c_.y);
  }


  //
  // accessors
  //

  __host__ __device__ __forceinline__ const Vec32fC2& f() const {return f_;}
  __host__ __device__ __forceinline__ const Vec32fC2& c() const  {return c_;}

  __host__ __device__ __forceinline__ float fx() const  {return f_.x;}
  __host__ __device__ __forceinline__ float fy() const  {return f_.y;}
  __host__ __device__ __forceinline__ float cx() const  {return c_.x;}
  __host__ __device__ __forceinline__ float cy() const  {return c_.y;}

private:
  Vec32fC2 f_; //!< focal length {fx, fy}
  Vec32fC2 c_; //!< principal point {cx, cy}

};

}
}

#endif // IMP_CU_PINHOLE_CAMERA_CUH

