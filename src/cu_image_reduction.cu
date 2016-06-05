#include <imp/cu_core/cu_image_reduction.cuh>
#include <imp/cu_core/cu_utils.hpp>

namespace ze {
namespace cu {

// From Reduction SDK sample:
// Prevent instantiation of the generic struct using an undefined symbol
// in the function body (so it won't compile)
template<typename Pixel>
struct SharedMemory
{
  __device__ Pixel *getPointer()
  {
    extern __device__ void error(void);
    error();
    return NULL;
  }
};

// Required specializations
template<>
struct SharedMemory<ze::Pixel32sC1>
{
  __device__ ze::Pixel32sC1 *getPointer()
  {
    extern __shared__ ze::Pixel32sC1 s_int[];
    return s_int;
  }
};

template<>
struct SharedMemory<ze::Pixel32fC1>
{
  __device__ Pixel32fC1 *getPointer()
  {
    extern __shared__ Pixel32fC1 s_float[];
    return s_float;
  }
};

// Templated kernels
template<typename Pixel>
__global__
void reductionSumKernel(
    Pixel* out_dev_ptr,
    ze::size_t out_stride,
    const Pixel* in_dev_ptr,
    ze::size_t in_stride,
    ze::uint32_t width,
    ze::uint32_t height)
{
  SharedMemory<Pixel> smem;
  Pixel* s_partial = smem.getPointer();

  Pixel sum = 0;

  // Sum over 2D thread grid, use (x,y) indices
  for (int x = blockIdx.x * blockDim.x + threadIdx.x;
       x < width;
       x += blockDim.x * gridDim.x)
  {
    for (int y = blockIdx.y * blockDim.y + threadIdx.y;
         y < height;
         y += blockDim.y * gridDim.y)
    {
      sum += in_dev_ptr[y*in_stride+x];
    }
  }
  // Sums are written to shared memory, single index
  s_partial[threadIdx.y*blockDim.x+threadIdx.x] = sum;
  __syncthreads();

  // Reduce over block sums stored in shared memory
  // Start using half the block threads,
  // halve the active threads at each iteration
  const int tid = threadIdx.y*blockDim.x+threadIdx.x;
  for (int num_active_threads = (blockDim.x*blockDim.y) >> 1;
       num_active_threads;
       num_active_threads >>= 1)
  {
    if (tid < num_active_threads)
    {
      s_partial[tid] += s_partial[tid+num_active_threads];
    }
    __syncthreads();
  }
  // Thread 0 writes the result for the block
  if(0 == tid)
  {
    out_dev_ptr[blockIdx.y*out_stride+blockIdx.x] = s_partial[0];
  }
}


template<typename Pixel>
__global__
void reductionCountEqKernel(
    Pixel* out_dev_ptr,
    ze::size_t out_stride,
    const Pixel* in_dev_ptr,
    size_t in_stride,
    ze::uint32_t width,
    ze::uint32_t height,
    Pixel value)
{
  SharedMemory<Pixel> smem;
  Pixel* s_partial = smem.getPointer();

  int32_t count = 0;

  // Sum over 2D thread grid, use (x,y) indices
  for (int x = blockIdx.x * blockDim.x + threadIdx.x;
       x < width;
       x += blockDim.x * gridDim.x)
  {
    for (int y = blockIdx.y * blockDim.y + threadIdx.y;
         y < height;
         y += blockDim.y * gridDim.y)
    {
      if(static_cast<int32_t>(value) == in_dev_ptr[y*in_stride+x])
      {
        count += 1;
      }
    }
  }
  // Sums are written to shared memory, single index
  s_partial[threadIdx.y*blockDim.x+threadIdx.x] = count;
  __syncthreads();

  // Reduce over block sums stored in shared memory
  // Start using half the block threads,
  // halve the active threads at each iteration
  const int tid = threadIdx.y*blockDim.x+threadIdx.x;
  for (int num_active_threads = (blockDim.x*blockDim.y) >> 1;
       num_active_threads;
       num_active_threads >>= 1 )
  {
    if (tid < num_active_threads)
    {
      s_partial[tid] += s_partial[tid+num_active_threads];
    }
    __syncthreads();
  }
  // Thread 0 writes the result for the block
  if(0 == tid)
  {
    out_dev_ptr[blockIdx.y*out_stride+blockIdx.x] = s_partial[0];
  }
}


template<typename Pixel>
ImageReducer<Pixel>::ImageReducer()
  : fragm_(dim3(4, 4, 1), dim3(16, 16, 1))
  , partial_(fragm_.dimGrid.x, fragm_.dimGrid.y)
{
  // Compute required amount of shared memory
  sh_mem_size_ = fragm_.dimBlock.x * fragm_.dimBlock.y * sizeof(Pixel);

  // Allocate final result
  dev_final_ = ze::cu::MemoryStorage<Pixel>::alloc(1);
}

template<typename Pixel>
ImageReducer<Pixel>::~ImageReducer()
{
  ze::cu::MemoryStorage<Pixel>::free(dev_final_);
}

// Sum image by reduction
// Cfr. listing 12.1 by N. Wilt, "The CUDA Handbook"
template<typename Pixel>
Pixel ImageReducer<Pixel>::sum(const ze::cu::ImageGpu<Pixel>& in_img)
{
  //if(is_dev_fin_alloc_ && is_dev_part_alloc_)

  reductionSumKernel<Pixel>
      <<<fragm_.dimGrid, fragm_.dimBlock, sh_mem_size_>>>
                                                        (partial_.data(),
                                                         partial_.stride(),
                                                         in_img.data(),
                                                         in_img.stride(),
                                                         in_img.width(),
                                                         in_img.height());

  reductionSumKernel<Pixel>
      <<<1, fragm_.dimBlock, sh_mem_size_>>>
                                           (dev_final_,
                                            0,
                                            partial_.data(),
                                            partial_.stride(),
                                            fragm_.dimGrid.x,
                                            fragm_.dimGrid.y);

  // download sum
  Pixel h_sum;
  const cudaError err =
      cudaMemcpy(
        &h_sum,
        dev_final_,
        sizeof(Pixel),
        cudaMemcpyDeviceToHost);
  if(cudaSuccess != err)
  {
    LOG(FATAL) << "sum: unable to copy result from device to host";
  }
  return h_sum;
}

// Count elements equal to 'value'
// First count over the thread grid,
// then perform a reduction sum on a single thread block
template<>
size_t ImageReducer<ze::Pixel32sC1>::countEqual(
    const ze::cu::ImageGpu32sC1& in_img,
    int32_t value)
{

  reductionCountEqKernel<ze::Pixel32sC1>
      <<<fragm_.dimGrid, fragm_.dimBlock, sh_mem_size_>>>
                                                        (partial_.data(),
                                                         partial_.stride(),
                                                         in_img.data(),
                                                         in_img.stride(),
                                                         in_img.width(),
                                                         in_img.height(),
                                                         value);

  reductionSumKernel<ze::Pixel32sC1>
      <<<1, fragm_.dimBlock, sh_mem_size_>>>
                                           (dev_final_,
                                            0,
                                            partial_.data(),
                                            partial_.stride(),
                                            fragm_.dimGrid.x,
                                            fragm_.dimGrid.y);

  // download count
  int32_t h_count;
  const cudaError err =
      cudaMemcpy(
        &h_count,
        dev_final_,
        sizeof(int32_t),
        cudaMemcpyDeviceToHost);
  if(cudaSuccess != err)
  {
    LOG(FATAL) << "countEqual: unable to copy result from device to host";
  }
  return static_cast<size_t>(h_count);
}


template class ImageReducer<ze::Pixel32sC1>;
template class ImageReducer<ze::Pixel32fC1>;

} // cu namespace
} // ze namespace
