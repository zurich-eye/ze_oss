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
#ifndef IMP_CU_TEXTURE2D_CUH
#define IMP_CU_TEXTURE2D_CUH

#include <memory>
#include <cstring>
#include <cuda_runtime_api.h>
#include <ze/common/logging.hpp>
#include <imp/core/pixel.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/core/size.hpp>
#include <imp/core/types.hpp>

namespace ze {
namespace cu {

/**
 * @brief The Texture2D struct wrappes the cuda texture object
 */
struct Texture2D
{
  cudaTextureObject_t tex_object;
  __device__ __forceinline__ operator cudaTextureObject_t() const {return tex_object;}

  using Ptr = std::shared_ptr<Texture2D>;
  //  using UPtr = std::unique_ptr<Texture2D>;

  __host__ Texture2D()
    : tex_object(0)
  {
  }

  __host__ __device__ Texture2D(cudaTextureObject_t _tex_object)
    : tex_object(_tex_object)
  {
  }

  __host__ Texture2D(const void* data, uint32_t pitch,
                     cudaChannelFormatDesc channel_desc,
                     ze::Size2u size,
                     bool _normalized_coords = false,
                     cudaTextureFilterMode filter_mode = cudaFilterModePoint,
                     cudaTextureAddressMode address_mode = cudaAddressModeClamp,
                     cudaTextureReadMode read_mode = cudaReadModeElementType)
  {
    cudaResourceDesc tex_res;
    std::memset(&tex_res, 0, sizeof(tex_res));
    tex_res.resType = cudaResourceTypePitch2D;
    tex_res.res.pitch2D.width = size.width();
    tex_res.res.pitch2D.height = size.height();
    tex_res.res.pitch2D.pitchInBytes = pitch;
    tex_res.res.pitch2D.devPtr = const_cast<void*>(data);
    tex_res.res.pitch2D.desc = channel_desc;

    cudaTextureDesc tex_desc;
    std::memset(&tex_desc, 0, sizeof(tex_desc));
    tex_desc.normalizedCoords = (_normalized_coords==true) ? 1 : 0;
    tex_desc.filterMode = filter_mode;
    tex_desc.addressMode[0] = address_mode;
    tex_desc.addressMode[1] = address_mode;
    tex_desc.readMode = read_mode;

    cudaError_t err = cudaCreateTextureObject(&tex_object, &tex_res, &tex_desc, 0);
    CHECK_EQ(err, ::cudaSuccess);
  }

  __host__ virtual ~Texture2D()
  {
    cudaError_t err = cudaDestroyTextureObject(tex_object);
    CHECK_EQ(err, ::cudaSuccess);
  }

  // copy and asignment operator enforcing deep copy
  __host__ __device__
  Texture2D(const Texture2D& other)
    : tex_object(other.tex_object)
  {
      cudaTextureDesc texture_desc;
      cudaResourceViewDesc resource_view_desc;
      cudaResourceDesc resource_desc;

      cudaGetTextureObjectTextureDesc(&texture_desc, other.tex_object);
      cudaGetTextureObjectResourceViewDesc(&resource_view_desc, other.tex_object);
      cudaGetTextureObjectResourceDesc(&resource_desc, other.tex_object);

      cudaCreateTextureObject(&this->tex_object, &resource_desc, &texture_desc, &resource_view_desc);
  }
  __host__ __device__
  Texture2D& operator=(const Texture2D& other)
  {
    if  (this != &other)
    {
      cudaTextureDesc texture_desc;
      cudaResourceViewDesc resource_view_desc;
      cudaResourceDesc resource_desc;

      cudaGetTextureObjectTextureDesc(&texture_desc, other.tex_object);
      cudaGetTextureObjectResourceViewDesc(&resource_view_desc, other.tex_object);
      cudaGetTextureObjectResourceDesc(&resource_desc, other.tex_object);

      cudaCreateTextureObject(&this->tex_object, &resource_desc, &texture_desc, &resource_view_desc);
    }
    return *this;
  }
};


} // namespace cu
} // namespace ze

#endif // IMP_CU_TEXTURE2D_CUH

