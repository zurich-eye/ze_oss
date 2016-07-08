cmake_minimum_required(VERSION 2.8.3)
project(ze_imu_evaluation)

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
    include/ze/imu_evaluation/pre_integrator_base.hpp
    include/ze/imu_evaluation/pre_integration_runner.hpp
    include/ze/imu_evaluation/pre_integration_mc_runner.hpp
    include/ze/imu_evaluation/manifold_pre_integrator.hpp
    include/ze/imu_evaluation/quaternion_pre_integrator.hpp
    include/ze/imu_evaluation/imu_preintegration_parameters.hpp
    include/ze/imu_evaluation/preintegration_evaluation_node.hpp
    )

set(SOURCES
    src/manifold_pre_integrator.cpp
    src/quaternion_pre_integrator.cpp
    src/pre_integrator_base.cpp
    src/pre_integration_mc_runner.cpp
    src/pre_integration_runner.cpp
    )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

###############
# EXECUTABLES #
###############
cs_add_executable(preintegration_evaluation_node src/preintegration_evaluation_node.cpp)
target_link_libraries(preintegration_evaluation_node ${PROJECT_NAME} ${PYTHON_LIBRARIES})

##########
# GTESTS #
##########

catkin_add_gtest(test_preintegrators test/test_preintegrators.cpp)
target_link_libraries(test_preintegrators ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
