project(ze_common)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)


#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/common/buffer.h
  include/ze/common/buffer-inl.h
  include/ze/common/file_utils.h
  include/ze/common/macros.h
  include/ze/common/manifold.h
  include/ze/common/matrix.h
  include/ze/common/numerical_derivative.h
  include/ze/common/path_utils.h
  include/ze/common/sample.h
  include/ze/common/statistics.h
  include/ze/common/stl_utils.h
  include/ze/common/string_utils.h
  include/ze/common/test_entrypoint.h
  include/ze/common/test_manifold.h
  include/ze/common/test_utils.h
  include/ze/common/time.h
  include/ze/common/timer.h
  include/ze/common/transformation.h
  include/ze/common/types.h
  include/ze/common/yaml_serialization.h
  include/ze/common/csv_trajectory.h
  )

set(SOURCES
  src/sample.cpp
  src/test_utils.cpp
  src/csv_trajectory.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})


##########
# GTESTS #
##########
catkin_add_gtest(test_benchmark test/test_benchmark.cpp)
target_link_libraries(test_benchmark ${PROJECT_NAME})

catkin_add_gtest(test_test_utils test/test_test_utils.cpp)
target_link_libraries(test_test_utils ${PROJECT_NAME})

catkin_add_gtest(test_transformation test/test_transformation.cpp)
target_link_libraries(test_transformation ${PROJECT_NAME})

catkin_add_gtest(test_string_utils test/test_string_utils.cpp)
target_link_libraries(test_string_utils ${PROJECT_NAME})

catkin_add_gtest(test_buffer test/test_buffer.cpp)
target_link_libraries(test_buffer ${PROJECT_NAME})

catkin_add_gtest(test_manifold test/test_manifold.cpp)
target_link_libraries(test_manifold ${PROJECT_NAME})

catkin_add_gtest(test_numerical_derivative test/test_numerical_derivative.cpp)
target_link_libraries(test_numerical_derivative ${PROJECT_NAME})

catkin_add_gtest(test_statistics test/test_statistics.cpp)
target_link_libraries(test_statistics ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
