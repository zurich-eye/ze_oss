project(ze_cameras)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ZeSetup)


#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/cameras/camera.h
  include/ze/cameras/camera_impl.h
  include/ze/cameras/camera_models.h
  include/ze/cameras/camera_yaml_serialization.h
  )

set(SOURCES
  src/camera.cpp
  src/camera_yaml_serialization.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})


##########
# GTESTS #
##########
catkin_add_gtest(test_pinhole test/test_pinhole.cpp)
target_link_libraries(test_pinhole ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
