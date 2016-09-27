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
#pragma once

#include <imp/core/pixel.hpp>
#include <imp/core/pixel_enums.hpp>

namespace ze {
namespace cu {

//
// uchar
//
unsigned char* __host__ __device__ toCudaVectorType(ze::Pixel8uC1* buffer) __attribute__ ((unused));
uchar2* __host__ __device__ toCudaVectorType(ze::Pixel8uC2* buffer) __attribute__ ((unused));
uchar3* __host__ __device__ toCudaVectorType(ze::Pixel8uC3* buffer) __attribute__ ((unused));
uchar4* __host__ __device__ toCudaVectorType(ze::Pixel8uC4* buffer) __attribute__ ((unused));

//
// ushort
//
unsigned short* __host__ __device__ toCudaVectorType(ze::Pixel16uC1* buffer) __attribute__ ((unused));
ushort2* __host__ __device__ toCudaVectorType(ze::Pixel16uC2* buffer) __attribute__ ((unused));
ushort3* __host__ __device__ toCudaVectorType(ze::Pixel16uC3* buffer) __attribute__ ((unused));
ushort4* __host__ __device__ toCudaVectorType(ze::Pixel16uC4* buffer) __attribute__ ((unused));

//
// unsigned int
//
unsigned int* __host__ __device__ toCudaVectorType(ze::Pixel32uC1* buffer) __attribute__ ((unused));
uint2* __host__ __device__ toCudaVectorType(ze::Pixel32uC2* buffer) __attribute__ ((unused));
uint3* __host__ __device__ toCudaVectorType(ze::Pixel32uC3* buffer) __attribute__ ((unused));
uint4* __host__ __device__ toCudaVectorType(ze::Pixel32uC4* buffer) __attribute__ ((unused));

//
// int
//
int* __host__ __device__ toCudaVectorType(ze::Pixel32sC1* buffer) __attribute__ ((unused));
int2* __host__ __device__ toCudaVectorType(ze::Pixel32sC2* buffer) __attribute__ ((unused));
int3* __host__ __device__ toCudaVectorType(ze::Pixel32sC3* buffer) __attribute__ ((unused));
int4* __host__ __device__ toCudaVectorType(ze::Pixel32sC4* buffer) __attribute__ ((unused));

//
// float
//
float* __host__ __device__ toCudaVectorType(ze::Pixel32fC1* buffer) __attribute__ ((unused));
float2* __host__ __device__ toCudaVectorType(ze::Pixel32fC2* buffer) __attribute__ ((unused));
float3* __host__ __device__ toCudaVectorType(ze::Pixel32fC3* buffer) __attribute__ ((unused));
float4* __host__ __device__ toCudaVectorType(ze::Pixel32fC4* buffer) __attribute__ ((unused));

//------------------------------------------------------------------------------
//
// uchar
//
const unsigned char* __host__ __device__ toConstCudaVectorType(const ze::Pixel8uC1* buffer) __attribute__ ((unused));
const uchar2* __host__ __device__ toConstCudaVectorType(const ze::Pixel8uC2* buffer) __attribute__ ((unused));
const uchar3* __host__ __device__ toConstCudaVectorType(const ze::Pixel8uC3* buffer) __attribute__ ((unused));
const uchar4* __host__ __device__ toConstCudaVectorType(const ze::Pixel8uC4* buffer) __attribute__ ((unused));

//
// ushort
//
const unsigned short* __host__ __device__ toConstCudaVectorType(const ze::Pixel16uC1* buffer) __attribute__ ((unused));
const ushort2* __host__ __device__ toConstCudaVectorType(const ze::Pixel16uC2* buffer) __attribute__ ((unused));
const ushort3* __host__ __device__ toConstCudaVectorType(const ze::Pixel16uC3* buffer) __attribute__ ((unused));
const ushort4* __host__ __device__ toConstCudaVectorType(const ze::Pixel16uC4* buffer) __attribute__ ((unused));

//
// unsigned int
//
const unsigned int* __host__ __device__ toConstCudaVectorType(const ze::Pixel32uC1* buffer) __attribute__ ((unused));
const uint2* __host__ __device__ toConstCudaVectorType(const ze::Pixel32uC2* buffer) __attribute__ ((unused));
const uint3* __host__ __device__ toConstCudaVectorType(const ze::Pixel32uC3* buffer) __attribute__ ((unused));
const uint4* __host__ __device__ toConstCudaVectorType(const ze::Pixel32uC4* buffer) __attribute__ ((unused));


//
// int
//
const int* __host__ __device__ toConstCudaVectorType(const ze::Pixel32sC1* buffer) __attribute__ ((unused));
const int2* __host__ __device__ toConstCudaVectorType(const ze::Pixel32sC2* buffer) __attribute__ ((unused));
const int3* __host__ __device__ toConstCudaVectorType(const ze::Pixel32sC3* buffer) __attribute__ ((unused));
const int4* __host__ __device__ toConstCudaVectorType(const ze::Pixel32sC4* buffer) __attribute__ ((unused));

//
// float
//
const float* __host__ __device__ toConstCudaVectorType(const ze::Pixel32fC1* buffer) __attribute__ ((unused));
const float2* __host__ __device__ toConstCudaVectorType(const ze::Pixel32fC2* buffer) __attribute__ ((unused));
const float3* __host__ __device__ toConstCudaVectorType(const ze::Pixel32fC3* buffer) __attribute__ ((unused));
const float4* __host__ __device__ toConstCudaVectorType(const ze::Pixel32fC4* buffer) __attribute__ ((unused));


cudaChannelFormatDesc toCudaChannelFormatDesc(ze::PixelType pixel_type) __attribute__ ((unused));



} // namespace cu
} // namespace ze


