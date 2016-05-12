project(imp_bridge_opencv)
cmake_minimum_required(VERSION 2.8.0)

if(${CMAKE_MAJOR_VERSION} VERSION_GREATER 3.0)
  cmake_policy(SET CMP0054 OLD)
endif(${CMAKE_MAJOR_VERSION} VERSION_GREATER 3.0)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

find_package(OpenCV REQUIRED core highgui imgproc)
list(APPEND catkin_LIBRARIES ${OpenCV_LIBS})

set(HEADERS
  include/imp/bridge/opencv/cv_connector_pixel_types.hpp
  include/imp/bridge/opencv/image_cv.hpp
  include/imp/bridge/opencv/cv_bridge.hpp
  include/imp/bridge/opencv/cu_cv_bridge.hpp
  )

set(SOURCES
  src/cv_bridge.cpp
  src/cv_connector_pixel_types.cpp
  src/image_cv.cpp
  )

set(IMPL_FILES
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${IMPL_FILES})
#target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES} ${catkin_LIBRARIES})

###
### GTESTS
###

# TODO (MWE)

# catkin_add_gtest(${PROJECT_NAME}-test
#    test/test_main.cpp
#    test/todo.cpp
# )
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)

cs_install()
cs_export()
