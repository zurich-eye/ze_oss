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

find_package(ArrayFire REQUIRED)
include_directories(${ArrayFire_INCLUDE_DIRS})

set(HEADERS
  include/imp/bridge/af/image_af.hpp
  include/imp/bridge/af/fast_detector_af.hpp
  )

set(SOURCES
  src/image_af.cpp
  src/fast_detector_af.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${IMPL_FILES})
target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES} ${catkin_LIBRARIES} ${ArrayFire_LIBRARIES})

cs_add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME})

###
### GTESTS
###

catkin_add_gtest(${PROJECT_NAME}-test
    test/imp_bridge_af_test.cpp
 )
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)

cs_install()
cs_export()
