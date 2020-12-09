# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(FATAL_ERROR "Could not find messages which '/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/NodeTransform.msg' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [int32_t] in package [loam_velodyne_msg] with paths [['/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg']]")
message(STATUS "loam_velodyne_msg: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iloam_velodyne_msg:/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(loam_velodyne_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg" NAME_WE)
add_custom_target(_loam_velodyne_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "loam_velodyne_msg" "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(loam_velodyne_msg
  "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/loam_velodyne_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(loam_velodyne_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/loam_velodyne_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(loam_velodyne_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(loam_velodyne_msg_generate_messages loam_velodyne_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg" NAME_WE)
add_dependencies(loam_velodyne_msg_generate_messages_cpp _loam_velodyne_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(loam_velodyne_msg_gencpp)
add_dependencies(loam_velodyne_msg_gencpp loam_velodyne_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS loam_velodyne_msg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(loam_velodyne_msg
  "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/loam_velodyne_msg
)

### Generating Services

### Generating Module File
_generate_module_eus(loam_velodyne_msg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/loam_velodyne_msg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(loam_velodyne_msg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(loam_velodyne_msg_generate_messages loam_velodyne_msg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg" NAME_WE)
add_dependencies(loam_velodyne_msg_generate_messages_eus _loam_velodyne_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(loam_velodyne_msg_geneus)
add_dependencies(loam_velodyne_msg_geneus loam_velodyne_msg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS loam_velodyne_msg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(loam_velodyne_msg
  "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/loam_velodyne_msg
)

### Generating Services

### Generating Module File
_generate_module_lisp(loam_velodyne_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/loam_velodyne_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(loam_velodyne_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(loam_velodyne_msg_generate_messages loam_velodyne_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg" NAME_WE)
add_dependencies(loam_velodyne_msg_generate_messages_lisp _loam_velodyne_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(loam_velodyne_msg_genlisp)
add_dependencies(loam_velodyne_msg_genlisp loam_velodyne_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS loam_velodyne_msg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(loam_velodyne_msg
  "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/loam_velodyne_msg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(loam_velodyne_msg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/loam_velodyne_msg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(loam_velodyne_msg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(loam_velodyne_msg_generate_messages loam_velodyne_msg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg" NAME_WE)
add_dependencies(loam_velodyne_msg_generate_messages_nodejs _loam_velodyne_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(loam_velodyne_msg_gennodejs)
add_dependencies(loam_velodyne_msg_gennodejs loam_velodyne_msg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS loam_velodyne_msg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(loam_velodyne_msg
  "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/loam_velodyne_msg
)

### Generating Services

### Generating Module File
_generate_module_py(loam_velodyne_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/loam_velodyne_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(loam_velodyne_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(loam_velodyne_msg_generate_messages loam_velodyne_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/slam/catkin_velodyne/src/loam_velodyne_msg/msg/Heading.msg" NAME_WE)
add_dependencies(loam_velodyne_msg_generate_messages_py _loam_velodyne_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(loam_velodyne_msg_genpy)
add_dependencies(loam_velodyne_msg_genpy loam_velodyne_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS loam_velodyne_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/loam_velodyne_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/loam_velodyne_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(loam_velodyne_msg_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(loam_velodyne_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(loam_velodyne_msg_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/loam_velodyne_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/loam_velodyne_msg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(loam_velodyne_msg_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(loam_velodyne_msg_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(loam_velodyne_msg_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/loam_velodyne_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/loam_velodyne_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(loam_velodyne_msg_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(loam_velodyne_msg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(loam_velodyne_msg_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/loam_velodyne_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/loam_velodyne_msg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(loam_velodyne_msg_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(loam_velodyne_msg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(loam_velodyne_msg_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/loam_velodyne_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/loam_velodyne_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/loam_velodyne_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(loam_velodyne_msg_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(loam_velodyne_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(loam_velodyne_msg_generate_messages_py nav_msgs_generate_messages_py)
endif()
