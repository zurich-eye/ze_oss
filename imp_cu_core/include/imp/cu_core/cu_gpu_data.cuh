#ifndef IMP_CU_GPU_DATA_CUH
#define IMP_CU_GPU_DATA_CUH

#include <imp/core/types.hpp>
#include <imp/core/roi.hpp>


namespace imp { namespace cu {

template<typename Pixel>
struct GpuData2D
{
  Pixel* data;
  std::uint32_t width;
  std::uint32_t height;
  imp::Roi2u roi;
  size_t stride;

  __host__ __device__
  GpuData2D(Pixel* _data, size_t _stride,
            std::uint32_t _width, std::uint32_t _height,
            imp::Roi2u _roi=imp::Roi2u())
    : data(_data)
    , stride(_stride)
    , width(_width)
    , height(_height)
    //, roi((_roi == imp::Roi2u()) ? imp::Roi2u(0u, 0u, _width, _height) : _roi)
  {
  }

  bool valid() { return (data != nullptr && width>0 && height >0); }

  void roiReset() { roi = imp::Roi2u(0,0,width,height); }

  __host__ __device__ __forceinline__
  bool inRoi(int x, int y)
  {
    if (roi != imp::Roi2u())
      return (x>=roi.x() && y>=roi.y() && x<roi.width() && y<roi.height());
    else
      return inBounds(x,y);
  }

  __host__ __device__ __forceinline__
  bool inBounds(int x, int y)
  {
    return (x>=0 && y>=0 && x<width && y<height);
  }

  __host__ __device__ __forceinline__
  size_t pitch()
  {
    return stride*sizeof(Pixel);
  }

  __host__ __device__ __forceinline__
  Pixel operator()(int x, int y)
  {
    return data[y*stride+x];
  }

  __host__ __device__ __forceinline__
  Pixel operator[](int x)
  {
    return data[x];
  }
};

} // namespace cu
} // namespace imp

#endif // IMP_CU_GPU_DATA_CUH

