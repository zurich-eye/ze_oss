project(imp_cu_correspondence)
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
  include/imp/cu_correspondence/variational_stereo.hpp
  include/imp/cu_correspondence/variational_epipolar_stereo.hpp
  include/imp/cu_correspondence/variational_stereo_parameters.hpp
  include/imp/cu_correspondence/stereo_solver_enum.hpp
  include/imp/cu_correspondence/stereo_ctf_warping.hpp
  include/imp/cu_correspondence/solver_stereo_abstract.hpp
  # TODO # include/imp/cu_correspondence/solver_epipolar_stereo_abstract.hpp
  )

set(SOURCES
  src/variational_stereo.cpp
  src/variational_epipolar_stereo.cpp
  src/stereo_ctf_warping.cpp
  )

set(CU_HDRS
  include/imp/cu_correspondence/solver_stereo_huber_l1.cuh
  include/imp/cu_correspondence/solver_stereo_precond_huber_l1.cuh
  include/imp/cu_correspondence/solver_stereo_precond_huber_l1_weighted.cuh
  include/imp/cu_correspondence/solver_epipolar_stereo_precond_huber_l1.cuh
  include/imp/cu_correspondence/occlusion.cuh
)


set(CU_SRCS
  src/warped_gradients_kernel.cuh

  src/solver_stereo_huber_l1_kernel.cuh
  src/solver_stereo_huber_l1.cu

  src/solver_precond_huber_l1_kernel.cuh
  src/solver_stereo_precond_huber_l1.cu

  src/solver_stereo_precond_huber_l1_weighted_kernel.cuh
  src/solver_stereo_precond_huber_l1_weighted.cu

  src/solver_epipolar_stereo_precond_huber_l1_kernel.cuh
  src/solver_epipolar_stereo_precond_huber_l1.cu

  src/occlusion.cu
  #src/occlusion_kernel.cuh
  )

#cuda_compile(CU_OBJ ${CU_SRCS} ${CU_HDRS})

#foreach(cu_file ${CU_SRCS})
#  message(STATUS "Process file: ${cu_file}")
#  set_source_files_properties(cu_file PROPERTIES HEADER_FILE_ONLY TRUE)
#endforeach()

cs_cuda_add_library(${PROJECT_NAME}
  ${CU_SRCS} ${CU_HDRS} ${SOURCES} ${HEADERS}
  )

cs_add_executable(main_node test/main_node.cpp)
target_link_libraries(main_node ${PROJECT_NAME})

##########
# GTESTS #
##########

catkin_add_gtest(test_dense_variational_stereo test/test_dense_variational_stereo.cpp)
target_link_libraries(test_dense_variational_stereo ${PROJECT_NAME})

catkin_add_gtest(test_dense_variational_epipolar_stereo test/test_dense_variational_epipolar_stereo.cpp)
target_link_libraries(test_dense_variational_epipolar_stereo ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
