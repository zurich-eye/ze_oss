project(ze_common)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)


#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/common/benchmark.h
  include/ze/common/buffer.h
  include/ze/common/buffer-inl.h
  include/ze/common/file_utils.h
  include/ze/common/logging.hpp
  include/ze/common/macros.h
  include/ze/common/manifold.h
  include/ze/common/matrix.h
  include/ze/common/nonassignable.hpp
  include/ze/common/noncopyable.hpp
  include/ze/common/numerical_derivative.h
  include/ze/common/path_utils.h
  include/ze/common/running_statistics.h
  include/ze/common/running_statistics_collection.h
  include/ze/common/sample.h
  include/ze/common/signal_handler.hpp
  include/ze/common/statistics.h
  include/ze/common/stl_utils.h
  include/ze/common/string_utils.h
  include/ze/common/test_entrypoint.h
  include/ze/common/test_manifold.h
  include/ze/common/test_utils.h
  include/ze/common/time_conversions.h
  include/ze/common/timer.h
  include/ze/common/timer_collection.h
  include/ze/common/timer_statistics.h
  include/ze/common/ts_match.h
  include/ze/common/transformation.h
  include/ze/common/thread_pool.h
  include/ze/common/types.h
  include/ze/common/yaml_serialization.h
  include/ze/common/csv_trajectory.h
  )

set(SOURCES
  src/csv_trajectory.cpp
  src/matrix.cpp
  src/sample.cpp
  src/signal_handler.cpp
  src/test_utils.cpp
  src/thread_pool.cpp
  src/timer_collection.cpp
  src/running_statistics_collection.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})


##########
# GTESTS #
##########
catkin_add_gtest(test_benchmark test/test_benchmark.cpp)
target_link_libraries(test_benchmark ${PROJECT_NAME})

catkin_add_gtest(test_buffer test/test_buffer.cpp)
target_link_libraries(test_buffer ${PROJECT_NAME})

catkin_add_gtest(test_manifold test/test_manifold.cpp)
target_link_libraries(test_manifold ${PROJECT_NAME})

catkin_add_gtest(test_matrix test/test_matrix.cpp)
target_link_libraries(test_matrix ${PROJECT_NAME})

catkin_add_gtest(test_numerical_derivative test/test_numerical_derivative.cpp)
target_link_libraries(test_numerical_derivative ${PROJECT_NAME})

catkin_add_gtest(test_running_statistics test/test_running_statistics.cpp)
target_link_libraries(test_running_statistics ${PROJECT_NAME})

catkin_add_gtest(test_statistics test/test_statistics.cpp)
target_link_libraries(test_statistics ${PROJECT_NAME})

catkin_add_gtest(test_string_utils test/test_string_utils.cpp)
target_link_libraries(test_string_utils ${PROJECT_NAME})

catkin_add_gtest(test_test_utils test/test_test_utils.cpp)
target_link_libraries(test_test_utils ${PROJECT_NAME})

catkin_add_gtest(test_timer test/test_timer.cpp)
target_link_libraries(test_timer ${PROJECT_NAME})

catkin_add_gtest(test_transformation test/test_transformation.cpp)
target_link_libraries(test_transformation ${PROJECT_NAME})

cs_add_executable(test_thread_pool test/test_thread_pool.cpp)
target_link_libraries(test_thread_pool ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
