project(ze_ros)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/ros/bag_image_fetcher.hpp
  include/ze/ros/pose_msg_bridge.h
  include/ze/ros/tf_bridge.h
  )

set(SOURCES
  src/bag_image_fetcher.cpp
  src/pose_msg_bridge.cpp
  src/tf_bridge.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########


##########
# EXPORT #
##########
cs_install()
cs_export()
