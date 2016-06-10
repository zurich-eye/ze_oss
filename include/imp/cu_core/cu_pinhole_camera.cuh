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

  template <typename T>
  __device__
  static void undistort(const T* params, T* px)
  {
    const T k1 = params[0];
    const T k2 = params[1];
    const T p1 = params[2];
    const T p2 = params[3];

    T jac_colmajor[4];
    T x[2];
    T x_tmp[2];
    x[0] = px[0]; x[1]= px[1];
    for(int i = 0; i < 30; ++i)
    {
      x_tmp[0] = x[0]; x_tmp[1] = x[1];
      distort(params, x_tmp, jac_colmajor);

      const T e_u = px[0] - x_tmp[0];
      const T e_v = px[1] - x_tmp[1];

      const T a = jac_colmajor[0];
      const T b = jac_colmajor[1];
      const T d = jac_colmajor[3];

      // direct gauss newton step
      const T a_sqr = a * a;
      const T b_sqr = b * b;
      const T d_sqr = d * d;
      const T abbd = a * b + b * d;
      const T abbd_sqr = abbd * abbd;
      const T a2b2 = a_sqr + b_sqr;
      const T a2b2_inv = 1.0 / a2b2;
      const T adabdb = a_sqr * d_sqr - 2 * a * b_sqr * d + b_sqr * b_sqr;
      const T adabdb_inv = 1.0 / adabdb;
      const T c1 = abbd * adabdb_inv;

      x[0] += e_u * (a * (abbd_sqr * a2b2_inv * adabdb_inv + a2b2_inv) - b * c1) + e_v * (b * (abbd_sqr * a2b2_inv * adabdb_inv + a2b2_inv) - d * c1);
      x[1] += e_u * (-a * c1 + b * a2b2 * adabdb_inv) + e_v * (-b * c1 + d * a2b2 * adabdb_inv);

      if ((e_u * e_u + e_v * e_v) < 1e-8)
      {
        break;
      }
    }

    px[0] = x[0];
    px[1] = x[1];
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

