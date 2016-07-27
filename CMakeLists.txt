project(ze_pangolin)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

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
target_link_libraries(ze_pangolin_test_node ${PROJECT_NAME})

##########
# GTESTS #
##########
catkin_add_gtest(test_type_watches test/test_type_watches.cpp)
target_link_libraries(test_type_watches ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
