project(ze_trajectory_analysis)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# This macro ensures modules and global scripts declared therein get installed
# See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

include(ze_setup)

###############
# EXECUTABLES #
###############
cs_add_executable(
	match_stamps
	src/match_stamps.cpp
)

##########
# GTESTS #
##########
catkin_add_gtest(test_ts_match test/test_ts_match.cpp)
target_link_libraries(test_ts_match ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
