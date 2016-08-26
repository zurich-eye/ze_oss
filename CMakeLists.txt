project(ze_trajectory_analysis)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# This macro ensures modules and global scripts declared therein get installed
# See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/trajectory_analysis/kitti_evaluation.hpp
  )

set(SOURCES
  src/kitti_evaluation.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

###############
# EXECUTABLES #
###############
cs_add_executable(match_stamps src/match_stamps_node.cpp)
target_link_libraries(match_stamps ${PROJECT_NAME})

cs_add_executable(kitti_evaluation src/kitti_evaluation_node.cpp)
target_link_libraries(kitti_evaluation ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
