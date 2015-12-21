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
  src/data_provider_csv.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# EXPORT #
##########
cs_install()
cs_export()
