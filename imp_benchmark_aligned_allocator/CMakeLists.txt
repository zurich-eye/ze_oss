project(imp_benchmark_aligned_allocator)
cmake_minimum_required(VERSION 2.8.0)

if(${CMAKE_MAJOR_VERSION} VERSION_GREATER 3.0)
  cmake_policy(SET CMP0054 OLD)
endif(${CMAKE_MAJOR_VERSION} VERSION_GREATER 3.0)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)
include(ze_macros_cuda)
find_cuda()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs" )

cs_add_executable(aligned_allocator_benchmark src/aligned_allocator_benchmark.cpp)
target_link_libraries(aligned_allocator_benchmark)

cs_install()
cs_export()
