#include <stdlib.h>
#include <iostream>
#include <cstdint>
#include <glog/logging.h>
#include <imp/core/timer.hpp>
#include <imp/core/image_raw.hpp>

int main(int argc, char* argv[])
{
  google::InitGoogleLogging(argv[0]);
  VLOG(2) << "Starting aligned allocator benchmarking";


  const size_t memory_size = 1e6;
  int memaddr_align = 32;
  std::uint64_t num_rounds = 1e8;

  {
    imp::SingleShotTimer timer("posix_memalign");

    for (std::uint64_t i=0; i<num_rounds; ++i)
    {
      std::uint8_t* p_data_aligned;
      int ret = posix_memalign((void**)&p_data_aligned, memaddr_align, memory_size);
      free(p_data_aligned);
      (void)ret;
    }

    LOG(INFO) << "posix_memalign: " << std::fixed << (double)timer.elapsedMs().count()/num_rounds << " ms / alloc+free";
  }

  {
    imp::SingleShotTimer timer("aligned_alloc");

    for (std::uint64_t i=0; i<num_rounds; ++i)
    {
      std::uint8_t* p_data_aligned = (std::uint8_t*)aligned_alloc(memaddr_align, memory_size);
      free(p_data_aligned);
    }

    LOG(INFO) << "aligned_alloc: " << std::fixed << (double)timer.elapsedMs().count()/num_rounds << " ms / alloc+free";
  }

}
