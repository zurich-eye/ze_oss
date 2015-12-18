project(ze_common)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ZeSetup)


#############
# LIBRARIES #
#############
set(HEADERS
	include/ze/common/path_utils.h
  include/ze/common/sample.h
  include/ze/common/string_utils.h
  include/ze/common/transformation.h
  include/ze/common/test/entrypoint.h
  )

set(SOURCES
  src/sample.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})


##########
# GTESTS #
##########
catkin_add_gtest(test_transformation test/test-transformation.cpp)
target_link_libraries(test_transformation ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
