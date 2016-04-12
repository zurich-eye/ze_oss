project(imp_tools_cmd)
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

###
### command line tools
###
# find_package( OpenCV REQUIRED core highgui imgproc)
# include_directories(${OpenCV_INCLUDE_DIRS})

cs_cuda_add_executable(image_filter_median3x3_test image_filter_median3x3_test.cpp)
# target_link_libraries(image_filter_median3x3_test)

cs_cuda_add_executable(image_filter_gauss_test image_filter_gauss_test.cpp)
# target_link_libraries(image_filter_gauss_test)

cs_cuda_add_executable(natural_edges_test natural_edges_test.cpp)
# target_link_libraries(natural_edges_test)

cs_cuda_add_executable(cu_rof_denoising_test cu_rof_denoising_test.cpp)
# target_link_libraries(cu_rof_denoising_test)

cs_cuda_add_executable(cu_tvl1_denoising_test cu_tvl1_denoising_test.cpp)
# target_link_libraries(cu_tvl1_denoising_test)

cuda_add_executable(cu_min_max_test cu_min_max_test.cpp)
target_link_libraries(cu_min_max_test ${CUDA_LIBRARIES} ${catkin_LIBRARIES})


#cs_add_executable(example_executable src/main.cc)
#target_link_libraries(example_executable example_library)

cs_install()
cs_export()
