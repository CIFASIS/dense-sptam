cmake_minimum_required(VERSION 2.8.3)
project(dense)

# Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

# Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# maximize optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")

# Find OpenCV library
#FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(OpenCV 2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Build the sptam library
FILE(GLOB DENSE_SRCS *.cpp)
add_library(dense ${DENSE_SRCS})
target_link_libraries(dense ${OpenCV_LIBRARIES} ${DLD_LIBRARIES} ${OPENGV_LIBRARY})
