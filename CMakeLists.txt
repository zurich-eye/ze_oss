project(imp_bridge_af)
cmake_minimum_required(VERSION 2.8.0)

if(${CMAKE_MAJOR_VERSION} VERSION_GREATER 3.0)
  cmake_policy(SET CMP0054 OLD)
endif(${CMAKE_MAJOR_VERSION} VERSION_GREATER 3.0)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)
include(ze_macros_cuda)
find_cuda()

FIND_LIBRARY(CUDA_NVVM_LIBRARY
  NAMES "nvvm"
  PATH_SUFFIXES "nvvm/lib64" "nvvm/lib"
  PATHS ${CUDA_TOOLKIT_ROOT_DIR}
  DOC "CUDA NVVM Library"
)

set(HEADERS
  include/imp/bridge/af/image_af.hpp
  include/imp/bridge/af/orb_detector_af.hpp
  )

set(SOURCES
  src/image_af.cpp
  src/orb_detector_af.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME}
  ${CUDA_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CUDA_NVVM_LIBRARY}
)

###
### GTESTS
###

catkin_add_gtest(${PROJECT_NAME}-test
    test/imp_bridge_af_test.cpp
 )
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)


cs_install()
cs_export()
