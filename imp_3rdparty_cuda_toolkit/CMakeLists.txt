cmake_minimum_required(VERSION 2.8.3)
project(imp_3rdparty_cuda_toolkit)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_macros_cuda)
find_cuda()


file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
message(STATUS "INSTALL FROM: ${CMAKE_CURRENT_SOURCE_DIR}/${CUDA_VERSION}/include")
message(STATUS "INSTALL TO:   ${CATKIN_DEVEL_PREFIX}/include")
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${CUDA_VERSION}/include/ DESTINATION ${CATKIN_DEVEL_PREFIX}/include
#    FILES_MATCHING PATTERN "*.h")
cs_add_library(${PROJECT_NAME} src/empty.cpp)

#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${CUDA_VERSION}/include)
cs_install()
cs_export(
  #   INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/imp_3rdparty_cuda_toolkit
  #   CFG_EXTRAS imp_3rdparty_cuda_toolkit-extras.cmake.in
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/${CUDA_VERSION}/include
  )
