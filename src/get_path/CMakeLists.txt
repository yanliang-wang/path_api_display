cmake_minimum_required(VERSION 2.8.3)
project(get_path)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

## Generate services in the 'srv' folder
add_service_files(FILES
  Refpointget.srv
  Pathget.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

