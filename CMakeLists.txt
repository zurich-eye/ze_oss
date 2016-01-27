cmake_minimum_required(VERSION 2.8.3)
project(ze_imu)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/imu/imu_calibration.h
  include/ze/imu/imu_calibration_yaml_serialization.h
  )

set(SOURCES
  src/imu_calibration.cpp
  src/imu_calibration_yaml_serialization.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
#catkin_add_gtest(test_pose_optimizer test/test_pose_optimizer.cpp)
#target_link_libraries(test_pose_optimizer ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
