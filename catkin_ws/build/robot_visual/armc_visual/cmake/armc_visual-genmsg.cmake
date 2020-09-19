# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "armc_visual: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(armc_visual_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv" NAME_WE)
add_custom_target(_armc_visual_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "armc_visual" "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv" ""
)

get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_armc_visual_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "armc_visual" "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(armc_visual
  "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/armc_visual
)
_generate_srv_cpp(armc_visual
  "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/armc_visual
)

### Generating Module File
_generate_module_cpp(armc_visual
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/armc_visual
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(armc_visual_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(armc_visual_generate_messages armc_visual_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv" NAME_WE)
add_dependencies(armc_visual_generate_messages_cpp _armc_visual_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(armc_visual_generate_messages_cpp _armc_visual_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(armc_visual_gencpp)
add_dependencies(armc_visual_gencpp armc_visual_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS armc_visual_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(armc_visual
  "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/armc_visual
)
_generate_srv_eus(armc_visual
  "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/armc_visual
)

### Generating Module File
_generate_module_eus(armc_visual
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/armc_visual
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(armc_visual_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(armc_visual_generate_messages armc_visual_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv" NAME_WE)
add_dependencies(armc_visual_generate_messages_eus _armc_visual_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(armc_visual_generate_messages_eus _armc_visual_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(armc_visual_geneus)
add_dependencies(armc_visual_geneus armc_visual_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS armc_visual_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(armc_visual
  "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/armc_visual
)
_generate_srv_lisp(armc_visual
  "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/armc_visual
)

### Generating Module File
_generate_module_lisp(armc_visual
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/armc_visual
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(armc_visual_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(armc_visual_generate_messages armc_visual_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv" NAME_WE)
add_dependencies(armc_visual_generate_messages_lisp _armc_visual_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(armc_visual_generate_messages_lisp _armc_visual_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(armc_visual_genlisp)
add_dependencies(armc_visual_genlisp armc_visual_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS armc_visual_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(armc_visual
  "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/armc_visual
)
_generate_srv_nodejs(armc_visual
  "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/armc_visual
)

### Generating Module File
_generate_module_nodejs(armc_visual
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/armc_visual
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(armc_visual_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(armc_visual_generate_messages armc_visual_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv" NAME_WE)
add_dependencies(armc_visual_generate_messages_nodejs _armc_visual_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(armc_visual_generate_messages_nodejs _armc_visual_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(armc_visual_gennodejs)
add_dependencies(armc_visual_gennodejs armc_visual_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS armc_visual_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(armc_visual
  "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armc_visual
)
_generate_srv_py(armc_visual
  "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armc_visual
)

### Generating Module File
_generate_module_py(armc_visual
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armc_visual
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(armc_visual_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(armc_visual_generate_messages armc_visual_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/VisualVar.srv" NAME_WE)
add_dependencies(armc_visual_generate_messages_py _armc_visual_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/d/catkin_ws/src/robot_visual/armc_visual/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(armc_visual_generate_messages_py _armc_visual_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(armc_visual_genpy)
add_dependencies(armc_visual_genpy armc_visual_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS armc_visual_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/armc_visual)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/armc_visual
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(armc_visual_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/armc_visual)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/armc_visual
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(armc_visual_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/armc_visual)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/armc_visual
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(armc_visual_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/armc_visual)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/armc_visual
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(armc_visual_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armc_visual)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armc_visual\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armc_visual
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(armc_visual_generate_messages_py std_msgs_generate_messages_py)
endif()
