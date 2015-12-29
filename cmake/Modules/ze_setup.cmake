# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
set(CMAKE_CXX_FLAGS "-pthread")
set(CMAKE_CXX_FLAGS "-Wall -Werror -fPIC -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
  add_definitions(-DHAVE_FAST_NEON)
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -D_LINUX")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  message(AUTHOR_WARNING "For now, we do not support Windows.")
else()
  message(SEND_ERROR "Unknown or unsupported system.")
endif()


set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -ffast-math -fno-finite-math-only")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()
