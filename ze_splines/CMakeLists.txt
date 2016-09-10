project(ze_splines)
cmake_minimum_required(VERSION 2.8.3)

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
    include/ze/splines/bspline.hpp
    include/ze/splines/bspline_pose_minimal.hpp
    include/ze/splines/operators.hpp
    include/ze/splines/rotation_vector.hpp
    include/ze/splines/viz_splines.hpp
    )

set(SOURCES
    src/bspline.cpp
    src/bspline_pose_minimal.cpp
    src/viz_splines.cpp
    )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})

###############
# EXECUTABLES #
###############
cs_add_executable(viz_splines_test src/viz_splines_test.cpp)
target_link_libraries(viz_splines_test ${PROJECT_NAME} ${PYTHON_LIBRARIES})

##########
# GTESTS #
##########
catkin_add_gtest(test_bspline test/test_bspline.cpp)
target_link_libraries(test_bspline ${PROJECT_NAME})

catkin_add_gtest(test_bspline_pose_minimal test/test_bspline_pose_minimal.cpp)
target_link_libraries(test_bspline_pose_minimal ${PROJECT_NAME} ${PYTHON_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
