# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hybrid_bci: 6 messages, 0 services")

set(MSG_I_FLAGS "-Ihybrid_bci:/home/michele/catkin_ws/src/hybrid_bci/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hybrid_bci_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg" NAME_WE)
add_custom_target(_hybrid_bci_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hybrid_bci" "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg" "hybrid_bci/P300_person"
)

get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg" NAME_WE)
add_custom_target(_hybrid_bci_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hybrid_bci" "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg" ""
)

get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg" NAME_WE)
add_custom_target(_hybrid_bci_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hybrid_bci" "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg"
  "${MSG_I_FLAGS}"
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hybrid_bci
)
_generate_msg_cpp(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hybrid_bci
)
_generate_msg_cpp(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hybrid_bci
)

### Generating Services

### Generating Module File
_generate_module_cpp(hybrid_bci
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hybrid_bci
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hybrid_bci_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hybrid_bci_generate_messages hybrid_bci_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_cpp _hybrid_bci_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_cpp _hybrid_bci_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_cpp _hybrid_bci_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hybrid_bci_gencpp)
add_dependencies(hybrid_bci_gencpp hybrid_bci_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hybrid_bci_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg"
  "${MSG_I_FLAGS}"
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hybrid_bci
)
_generate_msg_eus(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hybrid_bci
)
_generate_msg_eus(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hybrid_bci
)

### Generating Services

### Generating Module File
_generate_module_eus(hybrid_bci
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hybrid_bci
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hybrid_bci_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hybrid_bci_generate_messages hybrid_bci_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_eus _hybrid_bci_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_eus _hybrid_bci_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_eus _hybrid_bci_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hybrid_bci_geneus)
add_dependencies(hybrid_bci_geneus hybrid_bci_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hybrid_bci_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg"
  "${MSG_I_FLAGS}"
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hybrid_bci
)
_generate_msg_lisp(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hybrid_bci
)
_generate_msg_lisp(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hybrid_bci
)

### Generating Services

### Generating Module File
_generate_module_lisp(hybrid_bci
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hybrid_bci
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hybrid_bci_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hybrid_bci_generate_messages hybrid_bci_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_lisp _hybrid_bci_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_lisp _hybrid_bci_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_lisp _hybrid_bci_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hybrid_bci_genlisp)
add_dependencies(hybrid_bci_genlisp hybrid_bci_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hybrid_bci_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg"
  "${MSG_I_FLAGS}"
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hybrid_bci
)
_generate_msg_nodejs(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hybrid_bci
)
_generate_msg_nodejs(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hybrid_bci
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hybrid_bci
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hybrid_bci
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hybrid_bci_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hybrid_bci_generate_messages hybrid_bci_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_nodejs _hybrid_bci_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_nodejs _hybrid_bci_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_nodejs _hybrid_bci_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hybrid_bci_gennodejs)
add_dependencies(hybrid_bci_gennodejs hybrid_bci_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hybrid_bci_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg"
  "${MSG_I_FLAGS}"
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hybrid_bci
)
_generate_msg_py(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hybrid_bci
)
_generate_msg_py(hybrid_bci
  "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hybrid_bci
)

### Generating Services

### Generating Module File
_generate_module_py(hybrid_bci
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hybrid_bci
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hybrid_bci_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hybrid_bci_generate_messages hybrid_bci_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_py _hybrid_bci_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/P300_person.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_py _hybrid_bci_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/michele/catkin_ws/src/hybrid_bci/msg/motorimagery.msg" NAME_WE)
add_dependencies(hybrid_bci_generate_messages_py _hybrid_bci_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hybrid_bci_genpy)
add_dependencies(hybrid_bci_genpy hybrid_bci_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hybrid_bci_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hybrid_bci)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hybrid_bci
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hybrid_bci_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hybrid_bci)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hybrid_bci
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hybrid_bci_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hybrid_bci)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hybrid_bci
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hybrid_bci_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hybrid_bci)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hybrid_bci
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hybrid_bci_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hybrid_bci)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hybrid_bci\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hybrid_bci
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hybrid_bci_generate_messages_py std_msgs_generate_messages_py)
endif()
