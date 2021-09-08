cmake_minimum_required(VERSION 2.8)
project(map_to_cloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation 
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport 
)

find_package(PCL REQUIRED)
find_package(OpenCV)
find_package(Threads)



add_message_files(FILES 
BoundingBox.msg
BoundingBoxes.msg
ObjectCount.msg
)

generate_messages(
DEPENDENCIES
geometry_msgs
sensor_msgs
std_msgs

)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(map_to_cloud src/map_to_cloud.cpp)
target_link_libraries(map_to_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(map_to_cloud ${catkin_EXPORTED_TARGETS})

























