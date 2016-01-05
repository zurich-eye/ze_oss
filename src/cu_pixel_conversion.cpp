#include <imp/cu_core/cu_pixel_conversion.hpp>

#include <cuda_runtime.h>
#include <imp/core/pixel.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/cu_core/cu_exception.hpp>

namespace imp {
namespace cu {

/*
 * Pixel arrays to cuda vector types
 */

//
// uchar
//
unsigned char* __host__ __device__ toCudaVectorType(imp::Pixel8uC1* buffer)
{
  return reinterpret_cast<unsigned char*>(buffer);
}
uchar2* __host__ __device__ toCudaVectorType(imp::Pixel8uC2* buffer)
{
  return reinterpret_cast<uchar2*>(buffer);
}
uchar3* __host__ __device__ toCudaVectorType(imp::Pixel8uC3* buffer)
{
  return reinterpret_cast<uchar3*>(buffer);
}
uchar4* __host__ __device__ toCudaVectorType(imp::Pixel8uC4* buffer)
{
  return reinterpret_cast<uchar4*>(buffer);
}

//
// ushort
//
unsigned short* __host__ __device__ toCudaVectorType(imp::Pixel16uC1* buffer)
{
  return reinterpret_cast<unsigned short*>(buffer);
}
ushort2* __host__ __device__ toCudaVectorType(imp::Pixel16uC2* buffer)
{
  return reinterpret_cast<ushort2*>(buffer);
}
ushort3* __host__ __device__ toCudaVectorType(imp::Pixel16uC3* buffer)
{
  return reinterpret_cast<ushort3*>(buffer);
}
ushort4* __host__ __device__ toCudaVectorType(imp::Pixel16uC4* buffer)
{
  return reinterpret_cast<ushort4*>(buffer);
}

//
// unsigned int
//
unsigned int* __host__ __device__ toCudaVectorType(imp::Pixel32uC1* buffer)
{
  return reinterpret_cast<unsigned int*>(buffer);
}
uint2* __host__ __device__ toCudaVectorType(imp::Pixel32uC2* buffer)
{
  return reinterpret_cast<uint2*>(buffer);
}
uint3* __host__ __device__ toCudaVectorType(imp::Pixel32uC3* buffer)
{
  return reinterpret_cast<uint3*>(buffer);
}
uint4* __host__ __device__ toCudaVectorType(imp::Pixel32uC4* buffer)
{
  return reinterpret_cast<uint4*>(buffer);
}

//
// int
//
int* __host__ __device__ toCudaVectorType(imp::Pixel32sC1* buffer)
{
  return reinterpret_cast<int*>(buffer);
}
int2* __host__ __device__ toCudaVectorType(imp::Pixel32sC2* buffer)
{
  return reinterpret_cast<int2*>(buffer);
}
int3* __host__ __device__ toCudaVectorType(imp::Pixel32sC3* buffer)
{
  return reinterpret_cast<int3*>(buffer);
}
int4* __host__ __device__ toCudaVectorType(imp::Pixel32sC4* buffer)
{
  return reinterpret_cast<int4*>(buffer);
}

//
// float
//
float* __host__ __device__ toCudaVectorType(imp::Pixel32fC1* buffer)
{
  return reinterpret_cast<float*>(buffer);
}
float2* __host__ __device__ toCudaVectorType(imp::Pixel32fC2* buffer)
{
  return reinterpret_cast<float2*>(buffer);
}
float3* __host__ __device__ toCudaVectorType(imp::Pixel32fC3* buffer)
{
  return reinterpret_cast<float3*>(buffer);
}
float4* __host__ __device__ toCudaVectorType(imp::Pixel32fC4* buffer)
{
  return reinterpret_cast<float4*>(buffer);
}

/*
 * Pixel arrays to CONST cuda vector types
 */

//
// uchar
//
const unsigned char* __host__ __device__ toConstCudaVectorType(const imp::Pixel8uC1* buffer)
{
  return reinterpret_cast<const unsigned char*>(buffer);
}
const uchar2* __host__ __device__ toConstCudaVectorType(const imp::Pixel8uC2* buffer)
{
  return reinterpret_cast<const uchar2*>(buffer);
}
const uchar3* __host__ __device__ toConstCudaVectorType(const imp::Pixel8uC3* buffer)
{
  return reinterpret_cast<const uchar3*>(buffer);
}
const uchar4* __host__ __device__ toConstCudaVectorType(const imp::Pixel8uC4* buffer)
{
  return reinterpret_cast<const uchar4*>(buffer);
}

//
// ushort
//
const unsigned short* __host__ __device__ toConstCudaVectorType(const imp::Pixel16uC1* buffer)
{
  return reinterpret_cast<const unsigned short*>(buffer);
}
const ushort2* __host__ __device__ toConstCudaVectorType(const imp::Pixel16uC2* buffer)
{
  return reinterpret_cast<const ushort2*>(buffer);
}
const ushort3* __host__ __device__ toConstCudaVectorType(const imp::Pixel16uC3* buffer)
{
  return reinterpret_cast<const ushort3*>(buffer);
}
const ushort4* __host__ __device__ toConstCudaVectorType(const imp::Pixel16uC4* buffer)
{
  return reinterpret_cast<const ushort4*>(buffer);
}

//
// unsigned int
//
const unsigned int* __host__ __device__ toConstCudaVectorType(const imp::Pixel32uC1* buffer)
{
  return reinterpret_cast<const unsigned int*>(buffer);
}
const uint2* __host__ __device__ toConstCudaVectorType(const imp::Pixel32uC2* buffer)
{
  return reinterpret_cast<const uint2*>(buffer);
}
const uint3* __host__ __device__ toConstCudaVectorType(const imp::Pixel32uC3* buffer)
{
  return reinterpret_cast<const uint3*>(buffer);
}
const uint4* __host__ __device__ toConstCudaVectorType(const imp::Pixel32uC4* buffer)
{
  return reinterpret_cast<const uint4*>(buffer);
}

//
// int
//
const int* __host__ __device__ toConstCudaVectorType(const imp::Pixel32sC1* buffer)
{
  return reinterpret_cast<const int*>(buffer);
}
const int2* __host__ __device__ toConstCudaVectorType(const imp::Pixel32sC2* buffer)
{
  return reinterpret_cast<const int2*>(buffer);
}
const int3* __host__ __device__ toConstCudaVectorType(const imp::Pixel32sC3* buffer)
{
  return reinterpret_cast<const int3*>(buffer);
}
const int4* __host__ __device__ toConstCudaVectorType(const imp::Pixel32sC4* buffer)
{
  return reinterpret_cast<const int4*>(buffer);
}

//
// float
//
const float* __host__ __device__ toConstCudaVectorType(const imp::Pixel32fC1* buffer)
{
  return reinterpret_cast<const float*>(buffer);
}
const float2* __host__ __device__ toConstCudaVectorType(const imp::Pixel32fC2* buffer)
{
  return reinterpret_cast<const float2*>(buffer);
}
const float3* __host__ __device__ toConstCudaVectorType(const imp::Pixel32fC3* buffer)
{
  return reinterpret_cast<const float3*>(buffer);
}
const float4* __host__ __device__ toConstCudaVectorType(const imp::Pixel32fC4* buffer)
{
  return reinterpret_cast<const float4*>(buffer);
}


//-----------------------------------------------------------------------------
cudaChannelFormatDesc toCudaChannelFormatDesc(imp::PixelType pixel_type)
{
  switch (pixel_type)
  {
  case imp::PixelType::i8uC1:
  return cudaCreateChannelDesc<unsigned char>();
  case imp::PixelType::i8uC2:
  return cudaCreateChannelDesc<uchar2>();
  case imp::PixelType::i8uC3:
  return cudaCreateChannelDesc<uchar3>();
  case imp::PixelType::i8uC4:
  return cudaCreateChannelDesc<uchar4>();
  case imp::PixelType::i16uC1:
  return cudaCreateChannelDesc<unsigned short>();
  case imp::PixelType::i16uC2:
  return cudaCreateChannelDesc<ushort2>();
  case imp::PixelType::i16uC3:
  return cudaCreateChannelDesc<ushort3>();
  case imp::PixelType::i16uC4:
  return cudaCreateChannelDesc<ushort4>();
  case imp::PixelType::i32sC1:
  return cudaCreateChannelDesc<int>();
  case imp::PixelType::i32sC2:
  return cudaCreateChannelDesc<int2>();
  case imp::PixelType::i32sC3:
  return cudaCreateChannelDesc<int3>();
  case imp::PixelType::i32sC4:
  return cudaCreateChannelDesc<int4>();
  case imp::PixelType::i32fC1:
  return cudaCreateChannelDesc<float>();
  case imp::PixelType::i32fC2:
  return cudaCreateChannelDesc<float2>();
  case imp::PixelType::i32fC3:
  return cudaCreateChannelDesc<float3>();
  case imp::PixelType::i32fC4:
  return cudaCreateChannelDesc<float4>();
  default:
    throw imp::cu::Exception("Pixel type not supported to generate a CUDA texture.",
                             __FILE__, __FUNCTION__, __LINE__);
  }
}



} // namespace cu
} // namespace imp



