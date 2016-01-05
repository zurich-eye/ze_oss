#ifndef IMP_CU_TEXTURE2D_CUH
#define IMP_CU_TEXTURE2D_CUH

#include <memory>
#include <cstring>
#include <cuda_runtime_api.h>
#include <imp/core/types.hpp>
#include <imp/core/size.hpp>
#include <imp/core/pixel.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/cu_core/cu_exception.hpp>

namespace imp {
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

  __host__ Texture2D(const void* data, size_t pitch,
                     cudaChannelFormatDesc channel_desc,
                     imp::Size2u size,
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
    if  (err != ::cudaSuccess)
    {
      throw imp::cu::Exception("Failed to create texture object", err,
                               __FILE__, __FUNCTION__, __LINE__);
    }
  }

  __host__ virtual ~Texture2D()
  {
    cudaError_t err = cudaDestroyTextureObject(tex_object);
    if  (err != ::cudaSuccess)
    {
      throw imp::cu::Exception("Failed to destroy texture object", err,
                               __FILE__, __FUNCTION__, __LINE__);
    }
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
} // namespace imp

#endif // IMP_CU_TEXTURE2D_CUH

