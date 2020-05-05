# Install script for directory: /home/d/catkin_ws/src/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/d/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/d/catkin_ws/build/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/catkin_generated/installspace/rqt_ethercat_test_plugin.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rqt_ethercat_test_plugin/cmake" TYPE FILE FILES
    "/home/d/catkin_ws/build/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/catkin_generated/installspace/rqt_ethercat_test_pluginConfig.cmake"
    "/home/d/catkin_ws/build/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/catkin_generated/installspace/rqt_ethercat_test_pluginConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rqt_ethercat_test_plugin" TYPE FILE FILES "/home/d/catkin_ws/src/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  include("/home/d/catkin_ws/build/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/d/catkin_ws/build/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/catkin_generated/installspace/rqt_ethercat_test_plugin.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rqt_ethercat_test_plugin/cmake" TYPE FILE FILES
    "/home/d/catkin_ws/build/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/catkin_generated/installspace/rqt_ethercat_test_pluginConfig.cmake"
    "/home/d/catkin_ws/build/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/catkin_generated/installspace/rqt_ethercat_test_pluginConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rqt_ethercat_test_plugin" TYPE FILE FILES "/home/d/catkin_ws/src/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rqt_ethercat_test_plugin" TYPE FILE FILES "/home/d/catkin_ws/src/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/plugin.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librqt_ethercat_test_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librqt_ethercat_test_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librqt_ethercat_test_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/d/catkin_ws/devel/lib/librqt_ethercat_test_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librqt_ethercat_test_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librqt_ethercat_test_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librqt_ethercat_test_plugin.so"
         OLD_RPATH "/opt/ros/kinetic/lib:/home/d/catkin_ws/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librqt_ethercat_test_plugin.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rqt_ethercat_test_plugin" TYPE DIRECTORY FILES "/home/d/catkin_ws/src/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/include/rqt_ethercat_test_plugin/")
endif()

