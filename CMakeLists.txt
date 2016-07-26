project(ze_pangolin)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(X11)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/pangolin/pangolin.hpp
  include/ze/pangolin/type_watches.hpp
  include/ze/pangolin/pangolin_insight.hpp
  )

set(SOURCES
    src/pangolin.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

###############
# EXECUTABLES #
###############
cs_add_executable(ze_pangolin_test_node src/pangolin_test_node.cpp)
target_link_libraries(ze_pangolin_test_node ${PROJECT_NAME} ${X11_LIBRARIES})

##########
# GTESTS #
##########
#catkin_add_gtest(test_aslam_conversion test/test_aslam_conversion.cpp)
#target_link_libraries(test_aslam_conversion ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
