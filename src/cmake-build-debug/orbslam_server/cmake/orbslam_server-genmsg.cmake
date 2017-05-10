# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "orbslam_server: 0 messages, 6 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(orbslam_server_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_save.srv" NAME_WE)
add_custom_target(_orbslam_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "orbslam_server" "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_save.srv" ""
)

get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_get.srv" NAME_WE)
add_custom_target(_orbslam_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "orbslam_server" "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_get.srv" ""
)

get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_save.srv" NAME_WE)
add_custom_target(_orbslam_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "orbslam_server" "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_save.srv" ""
)

get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_get.srv" NAME_WE)
add_custom_target(_orbslam_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "orbslam_server" "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_get.srv" ""
)

get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_save.srv" NAME_WE)
add_custom_target(_orbslam_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "orbslam_server" "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_save.srv" ""
)

get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_get.srv" NAME_WE)
add_custom_target(_orbslam_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "orbslam_server" "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_get.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam_server
)
_generate_srv_cpp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_get.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam_server
)
_generate_srv_cpp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam_server
)
_generate_srv_cpp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_get.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam_server
)
_generate_srv_cpp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam_server
)
_generate_srv_cpp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_get.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam_server
)

### Generating Module File
_generate_module_cpp(orbslam_server
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam_server
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(orbslam_server_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(orbslam_server_generate_messages orbslam_server_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_save.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_cpp _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_get.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_cpp _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_save.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_cpp _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_get.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_cpp _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_save.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_cpp _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_get.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_cpp _orbslam_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orbslam_server_gencpp)
add_dependencies(orbslam_server_gencpp orbslam_server_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orbslam_server_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam_server
)
_generate_srv_lisp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_get.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam_server
)
_generate_srv_lisp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam_server
)
_generate_srv_lisp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_get.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam_server
)
_generate_srv_lisp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam_server
)
_generate_srv_lisp(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_get.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam_server
)

### Generating Module File
_generate_module_lisp(orbslam_server
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam_server
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(orbslam_server_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(orbslam_server_generate_messages orbslam_server_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_save.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_lisp _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_get.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_lisp _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_save.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_lisp _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_get.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_lisp _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_save.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_lisp _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_get.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_lisp _orbslam_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orbslam_server_genlisp)
add_dependencies(orbslam_server_genlisp orbslam_server_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orbslam_server_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam_server
)
_generate_srv_py(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_get.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam_server
)
_generate_srv_py(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam_server
)
_generate_srv_py(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_get.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam_server
)
_generate_srv_py(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam_server
)
_generate_srv_py(orbslam_server
  "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_get.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam_server
)

### Generating Module File
_generate_module_py(orbslam_server
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam_server
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(orbslam_server_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(orbslam_server_generate_messages orbslam_server_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_save.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_py _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_get.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_py _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_save.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_py _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_pose_get.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_py _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_muilt_save.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_py _orbslam_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lifu/catkin_ws/src/orbslam_server/srv/orbslam_get.srv" NAME_WE)
add_dependencies(orbslam_server_generate_messages_py _orbslam_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orbslam_server_genpy)
add_dependencies(orbslam_server_genpy orbslam_server_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orbslam_server_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orbslam_server
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(orbslam_server_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orbslam_server
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(orbslam_server_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam_server)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam_server\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orbslam_server
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(orbslam_server_generate_messages_py std_msgs_generate_messages_py)
