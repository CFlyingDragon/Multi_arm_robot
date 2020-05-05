# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "testcan: 2 messages, 0 services")

set(MSG_I_FLAGS "-Itestcan:/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(testcan_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg" NAME_WE)
add_custom_target(_testcan_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "testcan" "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg" ""
)

get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg" NAME_WE)
add_custom_target(_testcan_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "testcan" "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(testcan
  "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testcan
)
_generate_msg_cpp(testcan
  "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testcan
)

### Generating Services

### Generating Module File
_generate_module_cpp(testcan
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testcan
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(testcan_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(testcan_generate_messages testcan_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg" NAME_WE)
add_dependencies(testcan_generate_messages_cpp _testcan_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg" NAME_WE)
add_dependencies(testcan_generate_messages_cpp _testcan_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testcan_gencpp)
add_dependencies(testcan_gencpp testcan_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testcan_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(testcan
  "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testcan
)
_generate_msg_eus(testcan
  "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testcan
)

### Generating Services

### Generating Module File
_generate_module_eus(testcan
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testcan
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(testcan_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(testcan_generate_messages testcan_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg" NAME_WE)
add_dependencies(testcan_generate_messages_eus _testcan_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg" NAME_WE)
add_dependencies(testcan_generate_messages_eus _testcan_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testcan_geneus)
add_dependencies(testcan_geneus testcan_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testcan_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(testcan
  "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testcan
)
_generate_msg_lisp(testcan
  "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testcan
)

### Generating Services

### Generating Module File
_generate_module_lisp(testcan
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testcan
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(testcan_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(testcan_generate_messages testcan_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg" NAME_WE)
add_dependencies(testcan_generate_messages_lisp _testcan_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg" NAME_WE)
add_dependencies(testcan_generate_messages_lisp _testcan_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testcan_genlisp)
add_dependencies(testcan_genlisp testcan_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testcan_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(testcan
  "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testcan
)
_generate_msg_nodejs(testcan
  "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testcan
)

### Generating Services

### Generating Module File
_generate_module_nodejs(testcan
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testcan
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(testcan_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(testcan_generate_messages testcan_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg" NAME_WE)
add_dependencies(testcan_generate_messages_nodejs _testcan_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg" NAME_WE)
add_dependencies(testcan_generate_messages_nodejs _testcan_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testcan_gennodejs)
add_dependencies(testcan_gennodejs testcan_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testcan_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(testcan
  "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testcan
)
_generate_msg_py(testcan
  "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testcan
)

### Generating Services

### Generating Module File
_generate_module_py(testcan
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testcan
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(testcan_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(testcan_generate_messages testcan_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/IpPos.msg" NAME_WE)
add_dependencies(testcan_generate_messages_py _testcan_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_driver/CANopen_v1-master/testcan/msg/Frame.msg" NAME_WE)
add_dependencies(testcan_generate_messages_py _testcan_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testcan_genpy)
add_dependencies(testcan_genpy testcan_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testcan_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testcan)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testcan
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(testcan_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testcan)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testcan
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(testcan_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testcan)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testcan
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(testcan_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testcan)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testcan
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(testcan_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testcan)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testcan\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testcan
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(testcan_generate_messages_py std_msgs_generate_messages_py)
endif()
