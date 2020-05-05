# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robot_python: 2 messages, 0 services")

set(MSG_I_FLAGS "-Irobot_python:/home/d/catkin_ws/src/robot_python/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robot_python_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/Frame.msg" NAME_WE)
add_custom_target(_robot_python_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_python" "/home/d/catkin_ws/src/robot_python/msg/Frame.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg" NAME_WE)
add_custom_target(_robot_python_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_python" "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robot_python
  "/home/d/catkin_ws/src/robot_python/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_python
)
_generate_msg_cpp(robot_python
  "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_python
)

### Generating Services

### Generating Module File
_generate_module_cpp(robot_python
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_python
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robot_python_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robot_python_generate_messages robot_python_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/Frame.msg" NAME_WE)
add_dependencies(robot_python_generate_messages_cpp _robot_python_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg" NAME_WE)
add_dependencies(robot_python_generate_messages_cpp _robot_python_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_python_gencpp)
add_dependencies(robot_python_gencpp robot_python_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_python_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robot_python
  "/home/d/catkin_ws/src/robot_python/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_python
)
_generate_msg_eus(robot_python
  "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_python
)

### Generating Services

### Generating Module File
_generate_module_eus(robot_python
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_python
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robot_python_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robot_python_generate_messages robot_python_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/Frame.msg" NAME_WE)
add_dependencies(robot_python_generate_messages_eus _robot_python_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg" NAME_WE)
add_dependencies(robot_python_generate_messages_eus _robot_python_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_python_geneus)
add_dependencies(robot_python_geneus robot_python_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_python_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robot_python
  "/home/d/catkin_ws/src/robot_python/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_python
)
_generate_msg_lisp(robot_python
  "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_python
)

### Generating Services

### Generating Module File
_generate_module_lisp(robot_python
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_python
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robot_python_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robot_python_generate_messages robot_python_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/Frame.msg" NAME_WE)
add_dependencies(robot_python_generate_messages_lisp _robot_python_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg" NAME_WE)
add_dependencies(robot_python_generate_messages_lisp _robot_python_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_python_genlisp)
add_dependencies(robot_python_genlisp robot_python_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_python_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robot_python
  "/home/d/catkin_ws/src/robot_python/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_python
)
_generate_msg_nodejs(robot_python
  "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_python
)

### Generating Services

### Generating Module File
_generate_module_nodejs(robot_python
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_python
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robot_python_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robot_python_generate_messages robot_python_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/Frame.msg" NAME_WE)
add_dependencies(robot_python_generate_messages_nodejs _robot_python_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg" NAME_WE)
add_dependencies(robot_python_generate_messages_nodejs _robot_python_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_python_gennodejs)
add_dependencies(robot_python_gennodejs robot_python_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_python_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robot_python
  "/home/d/catkin_ws/src/robot_python/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_python
)
_generate_msg_py(robot_python
  "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_python
)

### Generating Services

### Generating Module File
_generate_module_py(robot_python
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_python
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robot_python_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robot_python_generate_messages robot_python_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/Frame.msg" NAME_WE)
add_dependencies(robot_python_generate_messages_py _robot_python_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_python/msg/IpPos.msg" NAME_WE)
add_dependencies(robot_python_generate_messages_py _robot_python_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_python_genpy)
add_dependencies(robot_python_genpy robot_python_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_python_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_python)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_python
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robot_python_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_python)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_python
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robot_python_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_python)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_python
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robot_python_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_python)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_python
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robot_python_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_python)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_python\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_python
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_python
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_python/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robot_python_generate_messages_py std_msgs_generate_messages_py)
endif()
