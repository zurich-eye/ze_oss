cmake_minimum_required(VERSION 2.8.3)
project(ze_visualization)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/visualization/viz_common.h
  include/ze/visualization/viz_gtsam.h
  include/ze/visualization/viz_interface.h
  include/ze/visualization/viz_publisher_ros.h
  )

set(SOURCES
  src/viz_common.cpp
  src/viz_gtsam.cpp  
  src/viz_publisher_ros.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

###############
# EXECUTABLES #
###############
cs_add_executable(viz_test_node src/viz_test_node.cpp)
target_link_libraries(viz_test_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
