cmake_minimum_required(VERSION 2.8.3)
project(ze_geometry)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/geometry/align_points.h
  include/ze/geometry/align_poses.h
  include/ze/geometry/clam.h
  include/ze/geometry/lsq_solver.h
  include/ze/geometry/lsq_solver-inl.h
  include/ze/geometry/lsq_state.h
  include/ze/geometry/pose_optimizer.h
  include/ze/geometry/pose_prior.h
  include/ze/geometry/robust_cost.h
  include/ze/geometry/triangulation.h
  )

set(SOURCES
  src/clam.cpp
  src/align_points.cpp
  src/align_poses.cpp
  src/pose_optimizer.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
catkin_add_gtest(test_align_points test/test_align_points.cpp)
target_link_libraries(test_align_points ${PROJECT_NAME})

catkin_add_gtest(test_align_poses test/test_align_poses.cpp)
target_link_libraries(test_align_poses ${PROJECT_NAME})

catkin_add_gtest(test_clam test/test_clam.cpp)
target_link_libraries(test_clam ${PROJECT_NAME})

catkin_add_gtest(test_lsq_state test/test_lsq_state.cpp)
target_link_libraries(test_lsq_state ${PROJECT_NAME})

catkin_add_gtest(test_pose_optimizer test/test_pose_optimizer.cpp)
target_link_libraries(test_pose_optimizer ${PROJECT_NAME})

catkin_add_gtest(test_robust_cost test/test_robust_cost.cpp)
target_link_libraries(test_robust_cost ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
