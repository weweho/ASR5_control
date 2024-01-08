cmake_minimum_required(VERSION 3.0.2)
project(asr5_control)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES asr5_control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include/asr5_control
  include/Json
  include/end_device
  include/end_sensor
  ${catkin_INCLUDE_DIRS}
)

link_directories(
        lib
        ${catkin_LIB_DIRS}
)

##########asr5_control####################
#add_executable(asr5_control
#        src/asr5_control.cpp
#        src/asr5_control/websocket_endpoint.cpp
#        src/asr5_control/ar5.cpp
#        src/Json/common.c
#        src/Json/decoder.c
#        src/Json/encoder.c
#        src/Json/helper.c
#        src/Json/ieee754.c
#        src/Json/parser.c
#        )
#
#target_link_libraries(asr5_control
#        ${catkin_LIBRARIES}
#        )


##########end_device####################
add_executable(end_device
        src/end_device.cpp
        src/end_device/end_effector.cpp
        src/end_device/end_sensor.cpp
        src/end_device/end_putter.cpp
        src/end_device/file_operator.cpp
        src/end_device/fsm.cpp
        )

target_link_libraries(end_device
        ${catkin_LIBRARIES}
        controlcan
        )

##########sensor_node####################
add_executable(sensor_node
        src/sensor_node.cpp
        src/end_device/fsm.cpp
        )

target_link_libraries(sensor_node
        ${catkin_LIBRARIES}
        )

##########motor_node####################
add_executable(motor_node
        src/motor_node.cpp
        src/end_device/fsm.cpp
        )

target_link_libraries(motor_node
        ${catkin_LIBRARIES}
        controlcan
        )

##########putter_node####################
add_executable(putter_node
        src/putter_node.cpp
        src/end_device/fsm.cpp
        )

target_link_libraries(putter_node
        ${catkin_LIBRARIES}
        )