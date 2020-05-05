cmake_minimum_required(VERSION 2.8.3)

project(sensor_description)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  tf
  urdf
  xacro
		)

find_package(roslaunch)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
#  add_library(robot1_description
#    src/${PROJECT_NAME}/robot1_description.cpp
#  )

## Declare a cpp executable

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robot1_description_node robot1_description_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 
