project(ze_splines)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
    include/ze/splines/bspline.hpp
    include/ze/splines/bspline_pose_minimal.hpp
    include/ze/splines/operators.hpp
    include/ze/splines/rotation_vector.hpp
  )

set(SOURCES
    src/bspline.cpp
    src/bspline_pose_minimal.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
catkin_add_gtest(test_bspline test/test_bspline.cpp)
target_link_libraries(test_bspline ${PROJECT_NAME})

catkin_add_gtest(test_bspline_pose_minimal test/test_bspline_pose_minimal.cpp)
target_link_libraries(test_bspline_pose_minimal ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
