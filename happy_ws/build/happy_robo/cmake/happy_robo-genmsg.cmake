# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "happy_robo: 1 messages, 1 services")

set(MSG_I_FLAGS "-Ihappy_robo:/home/happy/happy_ws/src/happy_robot/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(happy_robo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg" NAME_WE)
add_custom_target(_happy_robo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "happy_robo" "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg" "geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/PoseStamped"
)

get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv" NAME_WE)
add_custom_target(_happy_robo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "happy_robo" "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(happy_robo
  "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/happy_robo
)

### Generating Services
_generate_srv_cpp(happy_robo
  "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/happy_robo
)

### Generating Module File
_generate_module_cpp(happy_robo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/happy_robo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(happy_robo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(happy_robo_generate_messages happy_robo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg" NAME_WE)
add_dependencies(happy_robo_generate_messages_cpp _happy_robo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv" NAME_WE)
add_dependencies(happy_robo_generate_messages_cpp _happy_robo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(happy_robo_gencpp)
add_dependencies(happy_robo_gencpp happy_robo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS happy_robo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(happy_robo
  "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/happy_robo
)

### Generating Services
_generate_srv_eus(happy_robo
  "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/happy_robo
)

### Generating Module File
_generate_module_eus(happy_robo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/happy_robo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(happy_robo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(happy_robo_generate_messages happy_robo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg" NAME_WE)
add_dependencies(happy_robo_generate_messages_eus _happy_robo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv" NAME_WE)
add_dependencies(happy_robo_generate_messages_eus _happy_robo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(happy_robo_geneus)
add_dependencies(happy_robo_geneus happy_robo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS happy_robo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(happy_robo
  "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/happy_robo
)

### Generating Services
_generate_srv_lisp(happy_robo
  "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/happy_robo
)

### Generating Module File
_generate_module_lisp(happy_robo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/happy_robo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(happy_robo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(happy_robo_generate_messages happy_robo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg" NAME_WE)
add_dependencies(happy_robo_generate_messages_lisp _happy_robo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv" NAME_WE)
add_dependencies(happy_robo_generate_messages_lisp _happy_robo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(happy_robo_genlisp)
add_dependencies(happy_robo_genlisp happy_robo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS happy_robo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(happy_robo
  "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/happy_robo
)

### Generating Services
_generate_srv_nodejs(happy_robo
  "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/happy_robo
)

### Generating Module File
_generate_module_nodejs(happy_robo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/happy_robo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(happy_robo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(happy_robo_generate_messages happy_robo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg" NAME_WE)
add_dependencies(happy_robo_generate_messages_nodejs _happy_robo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv" NAME_WE)
add_dependencies(happy_robo_generate_messages_nodejs _happy_robo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(happy_robo_gennodejs)
add_dependencies(happy_robo_gennodejs happy_robo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS happy_robo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(happy_robo
  "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/happy_robo
)

### Generating Services
_generate_srv_py(happy_robo
  "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/happy_robo
)

### Generating Module File
_generate_module_py(happy_robo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/happy_robo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(happy_robo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(happy_robo_generate_messages happy_robo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/msg/AlignAndGoalPoseStamped.msg" NAME_WE)
add_dependencies(happy_robo_generate_messages_py _happy_robo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/happy/happy_ws/src/happy_robot/srv/TriggerWithCommand.srv" NAME_WE)
add_dependencies(happy_robo_generate_messages_py _happy_robo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(happy_robo_genpy)
add_dependencies(happy_robo_genpy happy_robo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS happy_robo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/happy_robo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/happy_robo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(happy_robo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(happy_robo_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/happy_robo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/happy_robo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(happy_robo_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(happy_robo_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/happy_robo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/happy_robo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(happy_robo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(happy_robo_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/happy_robo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/happy_robo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(happy_robo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(happy_robo_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/happy_robo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/happy_robo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/happy_robo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(happy_robo_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(happy_robo_generate_messages_py geometry_msgs_generate_messages_py)
endif()
