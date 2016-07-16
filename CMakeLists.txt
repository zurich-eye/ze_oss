project(imp_cu_imgproc)
cmake_minimum_required(VERSION 2.8.0)

if(${CMAKE_MAJOR_VERSION} VERSION_GREATER 3.0)
  cmake_policy(SET CMP0054 OLD)
endif(${CMAKE_MAJOR_VERSION} VERSION_GREATER 3.0)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)
include(ze_macros_cuda)
find_cuda()

set(HEADERS
  )

set(SOURCES
  )

set(CU_HDRS
  include/imp/cu_imgproc/image_pyramid.cuh
  include/imp/cu_imgproc/cu_reduce.cuh
  include/imp/cu_imgproc/cu_resample.cuh
  include/imp/cu_imgproc/cu_image_filter.cuh
  include/imp/cu_imgproc/edge_detectors.cuh
  include/imp/cu_imgproc/cu_variational_denoising.cuh
  include/imp/cu_imgproc/cu_rof_denoising.cuh
  include/imp/cu_imgproc/cu_tvl1_denoising.cuh
  include/imp/cu_imgproc/str_tex_decomposer.hpp
  include/imp/cu_imgproc/cu_remap.cuh
  include/imp/cu_imgproc/cu_undistortion.cuh
  include/imp/cu_imgproc/cu_stereo_rectification.cuh
  )

set(CU_SRCS
  src/cu_reduce.cu
  src/cu_resample.cu
  src/cu_median3x3_filter.cu
  src/cu_gauss_filter.cu
  src/cu_natural_edges.cu
  src/cu_variational_denoising.cu
  src/cu_rof_denoising.cu
  src/cu_tvl1_denoising.cu
  src/str_tex_decomposer.cpp
  src/cu_undistortion.cu
  src/cu_stereo_rectification.cu
  )

#cuda_compile(CU_OBJ ${CU_SRCS} ${CU_HDRS})

cs_cuda_add_library(${PROJECT_NAME}
   ${CU_SRCS} ${CU_HDRS} ${SOURCES} ${HEADERS}
   )
#target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES} ${catkin_LIBRARIES})

##########
# GTESTS #
##########
catkin_add_gtest(test_cu_image_pyramid test/test_cu_image_pyramid.cpp)
target_link_libraries(test_cu_image_pyramid ${PROJECT_NAME})

catkin_add_gtest(test_cu_rof test/test_cu_rof.cpp)
target_link_libraries(test_cu_rof ${PROJECT_NAME})

catkin_add_gtest(test_cu_str_tex_decomp test/test_cu_str_tex_decomp.cpp)
target_link_libraries(test_cu_str_tex_decomp ${PROJECT_NAME})

catkin_add_gtest(test_cu_undistortion test/test_cu_undistortion.cpp)
target_link_libraries(test_cu_undistortion ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
