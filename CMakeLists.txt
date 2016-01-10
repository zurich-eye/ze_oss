cmake_minimum_required(VERSION 2.8.3)
project(ze_geometry)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/geometry/jacobians.h
  include/ze/geometry/least_squares_solver.h
  include/ze/geometry/least_squares_solver-inl.h
  include/ze/geometry/pose_optimizer.h
  include/ze/geometry/robust_cost.h
  )

set(SOURCES
  src/pose_optimizer.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
catkin_add_gtest(test_pose_optimizer test/test_pose_optimizer.cpp)
target_link_libraries(test_pose_optimizer ${PROJECT_NAME})

catkin_add_gtest(test_robust_cost test/test_robust_cost.cpp)
target_link_libraries(test_robust_cost ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
