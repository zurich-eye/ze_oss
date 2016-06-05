project(imp_cu_core)
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
  include/imp/cu_core/cu_utils.hpp
  include/imp/cu_core/cu_exception.hpp
  include/imp/cu_core/cu_pixel_conversion.hpp
  )

set(SOURCES
  src/cu_pixel_conversion.cpp
  )

set(CU_HDRS
  include/imp/cu_core/cu_memory_storage.cuh
  include/imp/cu_core/cu_linearmemory.cuh
  include/imp/cu_core/cu_image_gpu.cuh
  include/imp/cu_core/cu_texture2d.cuh
  include/imp/cu_core/cu_texture.cuh
  include/imp/cu_core/cu_math.cuh
  include/imp/cu_core/cu_k_setvalue.cuh
  include/imp/cu_core/cu_k_derivative.cuh
  include/imp/cu_core/cu_pinhole_camera.cuh
  include/imp/cu_core/cu_se3.cuh
  include/imp/cu_core/cu_matrix.cuh
  include/imp/cu_core/cu_image_reduction.cuh
  )

set(CU_SRCS
  src/cu_linearmemory.cu
  src/cu_image_gpu.cu
  src/min_max.cu
  src/sum.cu
  src/cu_image_reduction.cu
  src/weighted_sum.cu
  )

#cuda_compile(CU_OBJ ${CU_SRC})

cs_cuda_add_library(${PROJECT_NAME}
   ${CU_SRCS} ${CU_HDRS} ${SOURCES} ${HEADERS}
   )
#target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES} ${catkin_LIBRARIES})

###
### GTESTS
###

catkin_add_gtest(${PROJECT_NAME}-test
  test/test_main.cpp
  test/cu_linearmemory_test.cpp
  test/image_gpu_simple_test.cpp
  #test/image_gpu_test.cpp
  test/min_max_test.cpp
  test/sum_test.cpp
  test/reduction_test.cpp
)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)

catkin_add_gtest(test_weighted_sum test/test_weighted_sum.cpp)
target_link_libraries(test_weighted_sum ${PROJECT_NAME})

cs_install()
cs_export()
