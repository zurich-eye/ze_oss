cmake_minimum_required(VERSION 2.8.3)
project(ze_vi_simulation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
    include/ze/vi_simulation/camera_simulator.hpp
    include/ze/vi_simulation/camera_simulator_types.hpp
    include/ze/vi_simulation/imu_bias_simulator.hpp
    include/ze/vi_simulation/imu_simulator.hpp
    include/ze/vi_simulation/trajectory_simulator.hpp
    include/ze/vi_simulation/evaluation_tools.hpp
    include/ze/vi_simulation/vi_simulator.hpp
    )

set(SOURCES
    src/camera_simulator.cpp
    src/imu_bias_simulator.cpp
    src/vi_simulator.cpp
    )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########

catkin_add_gtest(test_camera_simulator test/test_camera_simulator.cpp)
target_link_libraries(test_camera_simulator ${PROJECT_NAME})

catkin_add_gtest(test_imu_bias_simulator test/test_imu_bias_simulator.cpp)
target_link_libraries(test_imu_bias_simulator ${PROJECT_NAME})

catkin_add_gtest(test_imu_simulator test/test_imu_simulator.cpp)
target_link_libraries(test_imu_simulator ${PROJECT_NAME})

catkin_add_gtest(test_trajectory_simulator test/test_trajectory_simulator.cpp)
target_link_libraries(test_trajectory_simulator ${PROJECT_NAME})

catkin_add_gtest(test_vi_simulator test/test_vi_simulator.cpp)
target_link_libraries(test_vi_simulator ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
