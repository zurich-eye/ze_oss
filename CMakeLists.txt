cmake_minimum_required(VERSION 2.8.3)
project(ze_vi_simulation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

# The matplotlib-cpp headers are linked against
find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
set(HEADERS
    include/ze/vi_simulation/imu_bias_simulator.hpp
    include/ze/vi_simulation/imu_simulator.hpp
    include/ze/vi_simulation/trajectory_simulator.hpp
    include/ze/vi_simulation/evaluation_tools.hpp
    )

set(SOURCES
    src/imu_bias_simulator.cpp
    )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########

catkin_add_gtest(test_imu_bias_simulator test/test_imu_bias_simulator.cpp)
target_link_libraries(test_imu_bias_simulator ${PROJECT_NAME})

catkin_add_gtest(test_imu_simulator test/test_imu_simulator.cpp)
target_link_libraries(test_imu_simulator ${PROJECT_NAME})

catkin_add_gtest(test_trajectory_simulator test/test_trajectory_simulator.cpp)
target_link_libraries(test_trajectory_simulator ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
