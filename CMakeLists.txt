# Minimum cmake version
cmake_minimum_required(VERSION 2.8.3)

# Create project
project(pointcloud_stitching)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    param_io
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
)

# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    param_io
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(
  ${PROJECT_NAME}
  src/PointCloudStitching.cpp
)

# Declare a cpp executable
add_executable(
  ${PROJECT_NAME}_node
  src/pointcloud_stitching_node.cpp
)

# Specify dependencies on automatically generated service files
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
