cmake_minimum_required(VERSION 2.8.3)
project(ze_visualization)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)
include(ze_gtsam_optional)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/visualization/viz_common.h
  include/ze/visualization/viz_interface.h
  include/ze/visualization/viz_ros.h
  include/ze/visualization/viz_ros_utils.h
  )

set(SOURCES
  src/viz_common.cpp 
  src/viz_ros.cpp
  )

if(${gtsam_catkin_FOUND})
  list(APPEND HEADERS include/ze/visualization/viz_gtsam.h)
  list(APPEND SOURCES src/viz_gtsam.cpp)
endif()

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

###############
# EXECUTABLES #
###############
cs_add_executable(viz_example_node src/viz_example_node.cpp)
target_link_libraries(viz_example_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
