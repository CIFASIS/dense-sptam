cmake_minimum_required(VERSION 2.8.3)
project(dense)

# Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

# Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# maximize optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")

# Build utils
add_executable(kitti_ground_truth kitti_ground_truth.cpp)
target_link_libraries(kitti_ground_truth ${DENSE_LIBRARIES})
add_executable(depth_maps_comparision depth_maps_comparision.cpp)
target_link_libraries(depth_maps_comparision ${DENSE_LIBRARIES})
