# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mirte_msgs: 5 messages, 16 services")

set(MSG_I_FLAGS "-Imirte_msgs:/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mirte_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv" "mirte_msgs/color"
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv" ""
)

get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv" NAME_WE)
add_custom_target(_mirte_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirte_msgs" "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_msg_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_msg_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_msg_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_msg_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)

### Generating Services
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv"
  "${MSG_I_FLAGS}"
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_cpp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
)

### Generating Module File
_generate_module_cpp(mirte_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mirte_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mirte_msgs_generate_messages mirte_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_cpp _mirte_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mirte_msgs_gencpp)
add_dependencies(mirte_msgs_gencpp mirte_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mirte_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_msg_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_msg_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_msg_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_msg_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)

### Generating Services
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv"
  "${MSG_I_FLAGS}"
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)
_generate_srv_eus(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
)

### Generating Module File
_generate_module_eus(mirte_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mirte_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mirte_msgs_generate_messages mirte_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_eus _mirte_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mirte_msgs_geneus)
add_dependencies(mirte_msgs_geneus mirte_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mirte_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_msg_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_msg_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_msg_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_msg_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)

### Generating Services
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv"
  "${MSG_I_FLAGS}"
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)
_generate_srv_lisp(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
)

### Generating Module File
_generate_module_lisp(mirte_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mirte_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mirte_msgs_generate_messages mirte_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_lisp _mirte_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mirte_msgs_genlisp)
add_dependencies(mirte_msgs_genlisp mirte_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mirte_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_msg_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_msg_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_msg_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_msg_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)

### Generating Services
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv"
  "${MSG_I_FLAGS}"
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)
_generate_srv_nodejs(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
)

### Generating Module File
_generate_module_nodejs(mirte_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mirte_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mirte_msgs_generate_messages mirte_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_nodejs _mirte_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mirte_msgs_gennodejs)
add_dependencies(mirte_msgs_gennodejs mirte_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mirte_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_msg_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_msg_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_msg_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_msg_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)

### Generating Services
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv"
  "${MSG_I_FLAGS}"
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)
_generate_srv_py(mirte_msgs
  "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
)

### Generating Module File
_generate_module_py(mirte_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mirte_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mirte_msgs_generate_messages mirte_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Encoder.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/color.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Intensity.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/IntensityDigital.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/msg/Keypad.msg" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetDistance.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensity.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetIntensityDigital.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetEncoder.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetKeypad.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinMode.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/GetPinValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetPinValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetLEDValue.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetOLEDImage.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetServoAngle.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_virtual_color.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/get_barcode.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/SetMotorSpeed.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Move.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tanyaspee/mdp/bullproof-tech/bullproof_ws/src/mirte-ros-packages/mirte_msgs/srv/Turn.srv" NAME_WE)
add_dependencies(mirte_msgs_generate_messages_py _mirte_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mirte_msgs_genpy)
add_dependencies(mirte_msgs_genpy mirte_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mirte_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirte_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mirte_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(mirte_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirte_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mirte_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(mirte_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirte_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mirte_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(mirte_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirte_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mirte_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(mirte_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirte_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mirte_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(mirte_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
