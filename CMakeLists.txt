project(ze_data_provider)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ZeSetup)


#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/data_provider/data_provider_base.h
  include/ze/data_provider/data_provider_csv.h
  )

set(SOURCES
  src/data_provider_base.cpp
  src/data_provider_csv.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
catkin_add_gtest(test_data_provider_csv test/test_data_provider_csv.cpp)
target_link_libraries(test_data_provider_csv ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
