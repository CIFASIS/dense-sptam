cmake_minimum_required(VERSION 2.8.3)
project(dense)

# Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations -Wno-reorder")

# Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# maximize optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")

# Build utils
add_executable(kitti_ground_truth kitti_ground_truth.cpp)
target_link_libraries(kitti_ground_truth ${DENSE_LIBRARIES})
add_executable(tsukuba_ground_truth tsukuba_ground_truth.cpp)
target_link_libraries(tsukuba_ground_truth ${DENSE_LIBRARIES})
add_executable(depth_maps_comparision depth_maps_comparision.cpp)
target_link_libraries(depth_maps_comparision ${DENSE_LIBRARIES})


# Build kitti_dmap_generator
FILE(GLOB KITTI_DMAP_GENERATOR_SRCS kitti_dmap_generator.cpp kitti_dmap.cpp)
add_executable(kitti_dmap_generator ${KITTI_DMAP_GENERATOR_SRCS})
target_link_libraries(kitti_dmap_generator dense ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
