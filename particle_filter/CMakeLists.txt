cmake_minimum_required(VERSION 2.8.3)
project(particle_filter)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES particle_filter range_lib
  CATKIN_DEPENDS message_runtime rospy roscpp sensor_msgs std_msgs nav_msgs geometry_msgs tf
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/particle_filter_node.cpp)

## Add cmake target dependencies of the executable
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} range_lib)

#################################
# range_lib 프로젝트 설정 추가 #
#################################

file(GLOB SRCS 
    "vendor/lodepng/lodepng.cpp"
    "include/RangeLib.h"
    "include/RangeUtils.h"
    "include/CudaRangeLib.h"
    "include/lru_cache.h"
)

file(GLOB CUDA_SRCS 
    "include/kernels.cu"
    "vendor/lodepng/lodepng.cpp"
    "include/RangeLib.h"
    "include/RangeUtils.h"
    "include/CudaRangeLib.h"
    "include/lru_cache.h"
)

option(WITH_CUDA "Compile CUDA ray cast variants" OFF)
set(CHUNK_SIZE 262144)
set(CHUNK_THREADS 256)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

SET(BASEPATH "${CMAKE_SOURCE_DIR}") 
add_definitions(-DBASEPATH=${BASEPATH})
add_subdirectory(${PROJECT_SOURCE_DIR}/vendor/gflags)

# Use the basepath as a search location for includes
include_directories(${BASEPATH})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/vendor)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "-march=native -ffast-math -fno-math-errno ${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_BUILD_TYPE Release)

if (WITH_CUDA)
    add_definitions(-DUSE_CUDA=1)
    add_definitions(-DCHUNK_SIZE=${CHUNK_SIZE})
    add_definitions(-DNUM_THREADS=${CHUNK_THREADS})

    find_package(CUDA REQUIRED)
    
    cuda_add_library(range_lib ${CUDA_SRCS})
else()
    add_definitions(-DUSE_CUDA=0)
    add_library(range_lib ${SRCS})
endif()

target_link_libraries(range_lib gflags pthread)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node range_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
