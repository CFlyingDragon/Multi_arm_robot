cmake_minimum_required(VERSION 2.8.3)

project(arm_hand)

find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    std_msgs)

catkin_package()

find_package(roslaunch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_command_interface_node src/joint_command_interface.cpp)

target_link_libraries(joint_command_interface_node
    ${catkin_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
