cmake_minimum_required(VERSION 2.8.3)

project(arm_description)

find_package(catkin REQUIRED
  robot_state_publisher
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
# add_library(robot1_description
#   src/${PROJECT_NAME}/robot1_description.cpp
# )

## Declare a cpp executable
add_executable(state_publisher_tutorials1 src/state_publisher.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robot1_description_node robot1_description_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(state_publisher_tutorials1
  ${catkin_LIBRARIES}
 )
 foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(joint_command_interface_node1 src/joint_command_interface.cpp)

target_link_libraries(joint_command_interface_node1
    ${catkin_LIBRARIES})
