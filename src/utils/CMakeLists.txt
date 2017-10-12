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
add_executable(depth_maps_comparison depth_maps_comparison.cpp)
target_link_libraries(depth_maps_comparison ${DENSE_LIBRARIES})

# Find yaml-cpp Library
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

add_library(ProgramOptions ProgramOptions.cpp)

add_library(kitti_dmap kitti_dmap.cpp)

# Build kitti_dmap_generator
FILE(GLOB KITTI_DMAP_GENERATOR_SRCS kitti_dmap_generator.cpp)
add_executable(kitti_dmap_generator ${KITTI_DMAP_GENERATOR_SRCS})
target_link_libraries(kitti_dmap_generator kitti_dmap dense ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ProgramOptions)

# Build point_cloud_profiling
FILE(GLOB POINT_CLOUD_PROFILING_SRCS point_cloud_profiling.cpp)
add_executable(point_cloud_profiling ${POINT_CLOUD_PROFILING_SRCS})
target_link_libraries(point_cloud_profiling dense ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
