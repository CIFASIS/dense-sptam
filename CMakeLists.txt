cmake_minimum_required(VERSION 2.8.3)
project(dense)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	cmake_modules
	std_msgs
	sensor_msgs
	geometry_msgs
	cv_bridge
	message_filters
	image_geometry
	pcl_ros
	nav_msgs
	tf2
	tf2_geometry_msgs
	tf2_ros
	image_transport
)
include_directories(${catkin_INCLUDE_DIRS})

# Find Boost Library
find_package(Boost COMPONENTS thread system regex REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

# Find yaml-cpp Library
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# Find OpenCV library
#FIND_PACKAGE(OpenCV REQUIRED) # OpenCV 3
FIND_PACKAGE(OpenCV 2 REQUIRED) # OpenCV 2
include_directories(${OpenCV_INCLUDE_DIRS})

message(${OpenCV_LIBRARIES})

# Find PCL library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dense
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge message_filters image_geometry pcl_ros nav_msgs tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Build the dense library
add_subdirectory(src/dense)

set(DENSE_LIBRARIES
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  dense
)

# Build the dense node
add_subdirectory(src/ros)
