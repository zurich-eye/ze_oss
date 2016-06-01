cmake_minimum_required(VERSION 2.8.3)
project(ze_imu)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

# This macro ensures modules and global scripts declared therein get installed
# See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/imu/scenario.h
  include/ze/imu/imu_bias.h
  include/ze/imu/scenario_runner.h
  )
set(SOURCES
    src/imu_bias.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########

catkin_add_gtest(test_imu_bias test/test_imu_bias.cpp)
target_link_libraries(test_imu_bias ${PROJECT_NAME})

catkin_add_gtest(test_scenario test/test_scenario.cpp)
target_link_libraries(test_scenario ${PROJECT_NAME})

catkin_add_gtest(test_scenario_runner test/test_scenario_runner.cpp)
target_link_libraries(test_scenario_runner ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
