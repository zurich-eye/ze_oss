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
  include/imp/cu_imgproc/image_pyramid.hpp
  )

set(SOURCES
  )

set(CU_HDRS
  include/imp/cu_imgproc/cu_reduce.cuh
  include/imp/cu_imgproc/cu_resample.cuh
  include/imp/cu_imgproc/cu_image_filter.cuh
  include/imp/cu_imgproc/edge_detectors.cuh
  include/imp/cu_imgproc/cu_variational_denoising.cuh
  include/imp/cu_imgproc/cu_rof_denoising.cuh
  include/imp/cu_imgproc/cu_tvl1_denoising.cuh
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
  )

#cuda_compile(CU_OBJ ${CU_SRCS} ${CU_HDRS})

cs_cuda_add_library(${PROJECT_NAME}
   ${CU_SRCS} ${CU_HDRS} ${SOURCES} ${HEADERS}
   )
#target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES} ${catkin_LIBRARIES})

###
### GTESTS
###

# catkin_add_gtest(${PROJECT_NAME}-test
#   test/test_main.cpp
#   test/size_test.cpp
# )
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)

cs_install()
cs_export()
