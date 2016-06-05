#include <imp/cu_core/cu_image_reduction.cuh>
#include <imp/cu_core/cu_utils.hpp>

namespace ze {
namespace cu {

// From Reduction SDK sample:
// Prevent instantiation of the generic struct using an undefined symbol
// in the function body (so it won't compile)
template<typename T>
struct SharedMemory
{
  __device__ T *getPointer()
  {
    extern __device__ void error(void);
    error();
    return NULL;
  }
};

// Required specializations
template<>
struct SharedMemory<int>
{
  __device__ int *getPointer()
  {
    extern __shared__ int s_int[];
    return s_int;
  }
};

template<>
struct SharedMemory<float>
{
  __device__ float *getPointer()
  {
    extern __shared__ float s_float[];
    return s_float;
  }
};

// Templated kernels
template<typename T>
__global__
void reductionSumKernel(
    T *out_dev_ptr,
    size_t out_stride,
    const T *in_dev_ptr,
    size_t in_stride,
    size_t n,
    size_t m)
{
  SharedMemory<T> smem;
  T *s_partial = smem.getPointer();

  T sum = 0;

  // Sum over 2D thread grid, use (x,y) indices
  for(int x = blockIdx.x * blockDim.x + threadIdx.x;
      x < n;
      x += blockDim.x*gridDim.x)
  {
    for(int y = blockIdx.y * blockDim.y + threadIdx.y;
        y < m;
        y += blockDim.y*gridDim.y)
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
  for (int num_active_threads = (blockDim.x*blockDim.y)>>1;
       num_active_threads;
       num_active_threads >>= 1 ) {
    if ( tid < num_active_threads)
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

template<typename T>
__global__
void reductionCountEqKernel(
    int *out_dev_ptr,
    size_t out_stride,
    const T *in_dev_ptr,
    size_t in_stride,
    size_t n,
    size_t m,
    T value)
{
  SharedMemory<int> smem;
  int *s_partial = smem.getPointer();

  int count = 0;

  // Sum over 2D thread grid, use (x,y) indices
  for(int x = blockIdx.x * blockDim.x + threadIdx.x;
      x < n;
      x += blockDim.x*gridDim.x)
  {
    for(int y = blockIdx.y * blockDim.y + threadIdx.y;
        y < m;
        y += blockDim.y*gridDim.y)
    {
      if(value == in_dev_ptr[y*in_stride+x])
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
  for (int num_active_threads = (blockDim.x*blockDim.y)>>1;
       num_active_threads;
       num_active_threads >>= 1 ) {
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

template<typename T>
ImageReducer<T>::ImageReducer()
  : fragm_(dim3(4, 4, 1), dim3(16, 16, 1))
  , is_dev_part_alloc_(false)
  , is_dev_fin_alloc_(false)
{
  // Compute required amount of shared memory
  sh_mem_size_ = fragm_.dimBlock.x * fragm_.dimBlock.y * sizeof(T);

  // Allocate intermediate result
  const cudaError part_alloc_err = cudaMallocPitch(
        &dev_partial_,
        &dev_partial_pitch_,
        fragm_.dimGrid.x*sizeof(T),
        fragm_.dimGrid.y);
  if(cudaSuccess != part_alloc_err)
  {
    IMP_THROW_EXCEPTION("ImageReducer: unable to allocate pitched device memory for partial results");
  }
  else
  {
    dev_partial_stride_ = dev_partial_pitch_ / sizeof(T);
    is_dev_part_alloc_ = true;
  }

  // Allocate final result
  const cudaError fin_alloc_err = cudaMalloc(&dev_final_, sizeof(T));
  if(cudaSuccess != fin_alloc_err)
  {
    IMP_THROW_EXCEPTION("ImageReducer: unable to allocate device memory for final result");
  }
  else
  {
    is_dev_fin_alloc_ = true;
  }
}

template<typename T>
ImageReducer<T>::~ImageReducer()
{
  // Free device memory
  if(is_dev_fin_alloc_)
  {
    const cudaError err = cudaFree(dev_final_);
    if(cudaSuccess != err)
      IMP_THROW_EXCEPTION("ImageReducer: unable to free device memory");
  }
  if(is_dev_part_alloc_)
  {
    const cudaError err = cudaFree(dev_partial_);
    if(cudaSuccess != err)
      IMP_THROW_EXCEPTION("ImageReducer: unable to free device memory");
  }
}

// Sum image by reduction
// Cfr. listing 12.1 by N. Wilt, "The CUDA Handbook"
template<typename T>
T ImageReducer<T>::sum(
    const T *in_img_data,
    size_t in_img_stride,
    size_t in_img_width,
    size_t in_img_height)
{
  if(is_dev_fin_alloc_ && is_dev_part_alloc_)
  {
    reductionSumKernel<T>
        <<<fragm_.dimGrid, fragm_.dimBlock, sh_mem_size_>>>
                                                          (dev_partial_,
                                                           dev_partial_stride_,
                                                           in_img_data,
                                                           in_img_stride,
                                                           in_img_width,
                                                           in_img_height);

    reductionSumKernel<T>
        <<<1, fragm_.dimBlock, sh_mem_size_>>>
                                             (dev_final_,
                                              0,
                                              dev_partial_,
                                              dev_partial_stride_,
                                              fragm_.dimGrid.x,
                                              fragm_.dimGrid.y);

    // download sum
    T h_count;
    const cudaError err =
        cudaMemcpy(
          &h_count,
          dev_final_,
          sizeof(T),
          cudaMemcpyDeviceToHost);
    if(cudaSuccess != err)
    {
      IMP_THROW_EXCEPTION("sum: unable to copy result from device to host");
    }
    return h_count;
  }
  else
  {
    return 0;
  }
}

// Count elements equal to 'value'
// First count over the thread grid,
// then perform a reduction sum on a single thread block
template<>
size_t ImageReducer<int>::countEqual(
    const int *in_img_data,
    size_t in_img_stride,
    size_t in_img_width,
    size_t in_img_height,
    int value)
{
  if(is_dev_fin_alloc_ && is_dev_part_alloc_)
  {
    reductionCountEqKernel<int>
        <<<fragm_.dimGrid, fragm_.dimBlock, sh_mem_size_>>>
                                                          (dev_partial_,
                                                           dev_partial_stride_,
                                                           in_img_data,
                                                           in_img_stride,
                                                           in_img_width,
                                                           in_img_height,
                                                           value);

    reductionSumKernel<int>
        <<<1, fragm_.dimBlock, sh_mem_size_>>>
                                             (dev_final_,
                                              0,
                                              dev_partial_,
                                              dev_partial_stride_,
                                              fragm_.dimGrid.x,
                                              fragm_.dimGrid.y);

    // download sum
    int h_count;
    const cudaError err =
        cudaMemcpy(
          &h_count,
          dev_final_,
          sizeof(int),
          cudaMemcpyDeviceToHost);
    if(cudaSuccess != err)
      IMP_THROW_EXCEPTION("countEqual: unable to copy result from device to host");

    return static_cast<size_t>(h_count);
  }
  else
  {
    return 0;
  }
}

template class ImageReducer<int>;
template class ImageReducer<float>;

} // cu namespace
} // ze namespace
