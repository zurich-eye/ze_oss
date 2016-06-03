cmake_minimum_required(VERSION 2.8.3)
project(ze_visualization)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ze_setup)
include(ze_gtsam)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/visualization/viz_common.h
  include/ze/visualization/viz_interface.h
  include/ze/visualization/viz_ros.h
  include/ze/visualization/viz_ros_utils.h
  include/ze/visualization/matplotlibcpp.hpp
  )

set(SOURCES
  src/viz_common.cpp 
  src/viz_ros.cpp
  )

if(ZE_USE_GTSAM)
  list(APPEND HEADERS include/ze/visualization/viz_gtsam.h)
  list(APPEND SOURCES src/viz_gtsam.cpp)
endif()

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

###############
# EXECUTABLES #
###############
cs_add_executable(viz_example_node src/viz_example_node.cpp)
target_link_libraries(viz_example_node ${PROJECT_NAME})

cs_add_executable(mpl_example_node src/mpl_example_node.cpp)
target_link_libraries(mpl_example_node ${PROJECT_NAME} python2.7)

##########
# EXPORT #
##########
cs_install()
cs_export()
