cmake_minimum_required(VERSION 3.0.2)
project(asr5_control)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

find_package(Qt5 REQUIRED COMPONENTS Core)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES asr5_control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include/asr5_control
  include/Json
  include/end_effector
  include/end_sensor
  ${catkin_INCLUDE_DIRS}
)

link_directories(
        lib
        ${catkin_LIB_DIRS}
)

##########asr5_control####################
add_executable(asr5_control
        src/asr5_control.cpp
        src/asr5_control/websocket_endpoint.cpp
        src/asr5_control/ar5.cpp
        src/Json/common.c
        src/Json/decoder.c
        src/Json/encoder.c
        src/Json/helper.c
        src/Json/ieee754.c
        src/Json/parser.c
        )

target_link_libraries(asr5_control
        ${catkin_LIBRARIES}
        )


##########end_effector####################
add_executable(end_effector
        src/end_effector.cpp
        src/end_effector/end_effector.cpp
        )

target_link_libraries(end_effector
        ${catkin_LIBRARIES}
        controlcan
        Qt5::Core
        )



##########end_sensor####################
add_executable(end_sensor
        src/end_sensor.cpp
        )

target_link_libraries(end_sensor
        ${catkin_LIBRARIES}
        )