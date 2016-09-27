// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include <imp/cu_core/cu_pixel_conversion.hpp>
#include <cuda_runtime.h>
#include <ze/common/logging.hpp>
#include <imp/core/pixel.hpp>
#include <imp/core/pixel_enums.hpp>

namespace ze {
namespace cu {

/*
 * Pixel arrays to cuda vector types
 */

//
// uchar
//
unsigned char* __host__ __device__ toCudaVectorType(ze::Pixel8uC1* buffer)
{
  return reinterpret_cast<unsigned char*>(buffer);
}
uchar2* __host__ __device__ toCudaVectorType(ze::Pixel8uC2* buffer)
{
  return reinterpret_cast<uchar2*>(buffer);
}
uchar3* __host__ __device__ toCudaVectorType(ze::Pixel8uC3* buffer)
{
  return reinterpret_cast<uchar3*>(buffer);
}
uchar4* __host__ __device__ toCudaVectorType(ze::Pixel8uC4* buffer)
{
  return reinterpret_cast<uchar4*>(buffer);
}

//
// ushort
//
unsigned short* __host__ __device__ toCudaVectorType(ze::Pixel16uC1* buffer)
{
  return reinterpret_cast<unsigned short*>(buffer);
}
ushort2* __host__ __device__ toCudaVectorType(ze::Pixel16uC2* buffer)
{
  return reinterpret_cast<ushort2*>(buffer);
}
ushort3* __host__ __device__ toCudaVectorType(ze::Pixel16uC3* buffer)
{
  return reinterpret_cast<ushort3*>(buffer);
}
ushort4* __host__ __device__ toCudaVectorType(ze::Pixel16uC4* buffer)
{
  return reinterpret_cast<ushort4*>(buffer);
}

//
// unsigned int
//
unsigned int* __host__ __device__ toCudaVectorType(ze::Pixel32uC1* buffer)
{
  return reinterpret_cast<unsigned int*>(buffer);
}
uint2* __host__ __device__ toCudaVectorType(ze::Pixel32uC2* buffer)
{
  return reinterpret_cast<uint2*>(buffer);
}
uint3* __host__ __device__ toCudaVectorType(ze::Pixel32uC3* buffer)
{
  return reinterpret_cast<uint3*>(buffer);
}
uint4* __host__ __device__ toCudaVectorType(ze::Pixel32uC4* buffer)
{
  return reinterpret_cast<uint4*>(buffer);
}

//
// int
//
int* __host__ __device__ toCudaVectorType(ze::Pixel32sC1* buffer)
{
  return reinterpret_cast<int*>(buffer);
}
int2* __host__ __device__ toCudaVectorType(ze::Pixel32sC2* buffer)
{
  return reinterpret_cast<int2*>(buffer);
}
int3* __host__ __device__ toCudaVectorType(ze::Pixel32sC3* buffer)
{
  return reinterpret_cast<int3*>(buffer);
}
int4* __host__ __device__ toCudaVectorType(ze::Pixel32sC4* buffer)
{
  return reinterpret_cast<int4*>(buffer);
}

//
// float
//
float* __host__ __device__ toCudaVectorType(ze::Pixel32fC1* buffer)
{
  return reinterpret_cast<float*>(buffer);
}
float2* __host__ __device__ toCudaVectorType(ze::Pixel32fC2* buffer)
{
  return reinterpret_cast<float2*>(buffer);
}
float3* __host__ __device__ toCudaVectorType(ze::Pixel32fC3* buffer)
{
  return reinterpret_cast<float3*>(buffer);
}
float4* __host__ __device__ toCudaVectorType(ze::Pixel32fC4* buffer)
{
  return reinterpret_cast<float4*>(buffer);
}

/*
 * Pixel arrays to CONST cuda vector types
 */

//
// uchar
//
const unsigned char* __host__ __device__ toConstCudaVectorType(const ze::Pixel8uC1* buffer)
{
  return reinterpret_cast<const unsigned char*>(buffer);
}
const uchar2* __host__ __device__ toConstCudaVectorType(const ze::Pixel8uC2* buffer)
{
  return reinterpret_cast<const uchar2*>(buffer);
}
const uchar3* __host__ __device__ toConstCudaVectorType(const ze::Pixel8uC3* buffer)
{
  return reinterpret_cast<const uchar3*>(buffer);
}
const uchar4* __host__ __device__ toConstCudaVectorType(const ze::Pixel8uC4* buffer)
{
  return reinterpret_cast<const uchar4*>(buffer);
}

//
// ushort
//
const unsigned short* __host__ __device__ toConstCudaVectorType(const ze::Pixel16uC1* buffer)
{
  return reinterpret_cast<const unsigned short*>(buffer);
}
const ushort2* __host__ __device__ toConstCudaVectorType(const ze::Pixel16uC2* buffer)
{
  return reinterpret_cast<const ushort2*>(buffer);
}
const ushort3* __host__ __device__ toConstCudaVectorType(const ze::Pixel16uC3* buffer)
{
  return reinterpret_cast<const ushort3*>(buffer);
}
const ushort4* __host__ __device__ toConstCudaVectorType(const ze::Pixel16uC4* buffer)
{
  return reinterpret_cast<const ushort4*>(buffer);
}

//
// unsigned int
//
const unsigned int* __host__ __device__ toConstCudaVectorType(const ze::Pixel32uC1* buffer)
{
  return reinterpret_cast<const unsigned int*>(buffer);
}
const uint2* __host__ __device__ toConstCudaVectorType(const ze::Pixel32uC2* buffer)
{
  return reinterpret_cast<const uint2*>(buffer);
}
const uint3* __host__ __device__ toConstCudaVectorType(const ze::Pixel32uC3* buffer)
{
  return reinterpret_cast<const uint3*>(buffer);
}
const uint4* __host__ __device__ toConstCudaVectorType(const ze::Pixel32uC4* buffer)
{
  return reinterpret_cast<const uint4*>(buffer);
}

//
// int
//
const int* __host__ __device__ toConstCudaVectorType(const ze::Pixel32sC1* buffer)
{
  return reinterpret_cast<const int*>(buffer);
}
const int2* __host__ __device__ toConstCudaVectorType(const ze::Pixel32sC2* buffer)
{
  return reinterpret_cast<const int2*>(buffer);
}
const int3* __host__ __device__ toConstCudaVectorType(const ze::Pixel32sC3* buffer)
{
  return reinterpret_cast<const int3*>(buffer);
}
const int4* __host__ __device__ toConstCudaVectorType(const ze::Pixel32sC4* buffer)
{
  return reinterpret_cast<const int4*>(buffer);
}

//
// float
//
const float* __host__ __device__ toConstCudaVectorType(const ze::Pixel32fC1* buffer)
{
  return reinterpret_cast<const float*>(buffer);
}
const float2* __host__ __device__ toConstCudaVectorType(const ze::Pixel32fC2* buffer)
{
  return reinterpret_cast<const float2*>(buffer);
}
const float3* __host__ __device__ toConstCudaVectorType(const ze::Pixel32fC3* buffer)
{
  return reinterpret_cast<const float3*>(buffer);
}
const float4* __host__ __device__ toConstCudaVectorType(const ze::Pixel32fC4* buffer)
{
  return reinterpret_cast<const float4*>(buffer);
}


//-----------------------------------------------------------------------------
cudaChannelFormatDesc toCudaChannelFormatDesc(ze::PixelType pixel_type)
{
  switch (pixel_type)
  {
  case ze::PixelType::i8uC1:
    return cudaCreateChannelDesc<unsigned char>();
  case ze::PixelType::i8uC2:
    return cudaCreateChannelDesc<uchar2>();
  case ze::PixelType::i8uC3:
    return cudaCreateChannelDesc<uchar3>();
  case ze::PixelType::i8uC4:
    return cudaCreateChannelDesc<uchar4>();
  case ze::PixelType::i16uC1:
    return cudaCreateChannelDesc<unsigned short>();
  case ze::PixelType::i16uC2:
    return cudaCreateChannelDesc<ushort2>();
  case ze::PixelType::i16uC3:
    return cudaCreateChannelDesc<ushort3>();
  case ze::PixelType::i16uC4:
    return cudaCreateChannelDesc<ushort4>();
  case ze::PixelType::i32sC1:
    return cudaCreateChannelDesc<int>();
  case ze::PixelType::i32sC2:
    return cudaCreateChannelDesc<int2>();
  case ze::PixelType::i32sC3:
    return cudaCreateChannelDesc<int3>();
  case ze::PixelType::i32sC4:
    return cudaCreateChannelDesc<int4>();
  case ze::PixelType::i32fC1:
    return cudaCreateChannelDesc<float>();
  case ze::PixelType::i32fC2:
    return cudaCreateChannelDesc<float2>();
  case ze::PixelType::i32fC3:
    return cudaCreateChannelDesc<float3>();
  case ze::PixelType::i32fC4:
    return cudaCreateChannelDesc<float4>();
  default:
    CHECK(false) << "Pixel type not supported to generate a CUDA texture.";
  }
}

} // namespace cu
} // namespace ze



