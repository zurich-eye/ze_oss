#ifndef IMP_CU_PINHOLE_CAMERA_CUH
#define IMP_CU_PINHOLE_CAMERA_CUH

#include <cuda_runtime_api.h>
#include <imp/core/pixel.hpp>
#include <imp/cu_core/cu_matrix.cuh>

namespace imp {
namespace cu {

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
  imp::cu::Matrix3f intrinsics()
  {
    imp::cu::Matrix3f K;
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

