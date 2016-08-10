cmake_minimum_required(VERSION 2.8.3)
project(dense)

# Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

# Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# maximize optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")

add_library(dense_nodelet denseNodelet.cpp denseInterface.cpp)
target_link_libraries(dense_nodelet ${DENSE_LIBRARIES})

add_executable(dense_node dense_node.cpp)
target_link_libraries(dense_node ${DENSE_LIBRARIES})
