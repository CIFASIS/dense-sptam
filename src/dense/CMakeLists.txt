cmake_minimum_required(VERSION 2.8.3)
project(dense)

# Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations -Wno-reorder")

# Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# maximize optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")

# Build the sptam library
FILE(GLOB DENSE_SRCS *.cpp)
add_library(dense ${DENSE_SRCS})
target_link_libraries(dense ${DLD_LIBRARIES} elas kitti_dmap)
