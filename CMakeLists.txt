cmake_minimum_required(VERSION 2.8.3)
project(ze_imu)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/imu/imu_yaml_serialization.h
  include/ze/imu/imu.h
  include/ze/imu/imu_factory.h
  include/ze/imu/imu_distortion.h
  include/ze/imu/imu_distortion_scale_misalignment.h
  )

set(SOURCES
  src/imu_yaml_serialization.cpp
  src/imu.cpp
  src/imu_factory.cpp
  src/imu_distortion.cpp
  src/imu_distortion_scale_misalignment.cpp
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
