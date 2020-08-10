# Install script for directory: /home/d/catkin_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/d/catkin_ws/debug/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/d/catkin_ws/debug/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash;/usr/local/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/d/catkin_ws/debug/catkin_generated/installspace/setup.bash"
    "/home/d/catkin_ws/debug/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh;/usr/local/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/d/catkin_ws/debug/catkin_generated/installspace/setup.sh"
    "/home/d/catkin_ws/debug/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh;/usr/local/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/d/catkin_ws/debug/catkin_generated/installspace/setup.zsh"
    "/home/d/catkin_ws/debug/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/d/catkin_ws/debug/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/d/catkin_ws/debug/gtest/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/c_lightlobot/c_lightcobot/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_description/finger/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_dependent/hrl-kdl/hrl_kdl/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_description/robot_description/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_driver/robot_drive/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gazebo/robot_gazebo/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gui/robot_gui/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_kinematic/robot_kinematic/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_moveit/robot_moveit/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_sensor/robot_sensor/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gazebo/gazebo_ros_demos/rrbot_control/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gazebo/gazebo_ros_demos/rrbot_description/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gazebo/gazebo_ros_demos/rrbot_gazebo/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_description/sensor_description/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_driver/CANopen_v1-master/testcan/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_description/ur_description/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_dependent/hrl-kdl/hrl_geom/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_kinematic/kinematics/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gui/python_gui/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_msg/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_python/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_description/arm_hand/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_controller/forward_command_controller/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_controller/position_controllers/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_sensor/force_sensor/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gazebo/armc_gazebo/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/c_lightlobot/armc_gazebo1/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gazebo/armt_gazebo/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gazebo/gazebo_ros_demos/gazebo_tutorials/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_dependent/hrl-kdl/pykdl_utils/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gazebo/robots_gazebo/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gazebo/ur_gazebo/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_kinematic/kdl_kinematic/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_controller/impedance_controller/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_driver/ros_ethercat-master/ros_ethercat_driver/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_controller/armc_controller/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_gazebo/gazebo_ros_demos/rrbot_moveit_config/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_description/two_link/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/c_lightlobot/arm_description/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_description/armc_description/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_moveit/armc_moveit/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/c_lightlobot/armc_moveit_config/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_moveit/armc_moveit_config1/cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_description/armt_description /cmake_install.cmake")
  include("/home/d/catkin_ws/debug/robot_description/unknown_surface/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/d/catkin_ws/debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
