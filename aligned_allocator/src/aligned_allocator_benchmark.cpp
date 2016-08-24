#include <cstdint>
#include <iostream>
#include <stdlib.h>
#include <ze/common/logging.hpp>
#include <ze/common/timer.h>
#include <ze/common/timer_statistics.h>
#include <imp/core/image_raw.hpp>

int main(int argc, char* argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  VLOG(1) << "Starting aligned allocator benchmarking";


  const size_t memory_size = 1e6;
  int memaddr_align = 32;
  std::uint64_t num_rounds = 1e6;

  {
    ze::TimerStatistics timer;
    for (std::uint64_t i=0; i<num_rounds; ++i)
    {
      __attribute__((unused)) auto t = timer.timeScope();
      std::uint8_t* p_data_aligned;
      int ret = posix_memalign((void**)&p_data_aligned, memaddr_align, memory_size);
      free(p_data_aligned);
      (void)ret;
    }
    VLOG(1) << "posix_memalign: " << timer.mean() << "ms";
  }

  {
    ze::TimerStatistics timer;
    for (std::uint64_t i=0; i<num_rounds; ++i)
    {
      __attribute__((unused)) auto t = timer.timeScope();
      std::uint8_t* p_data_aligned = (std::uint8_t*)aligned_alloc(memaddr_align, memory_size);
      free(p_data_aligned);
    }
    VLOG(1) << "posix_memalign: " << timer.mean() << "ms";
  }

}
