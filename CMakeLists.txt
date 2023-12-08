cmake_minimum_required(VERSION 3.0.2)
project(asr5_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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
  ${catkin_INCLUDE_DIRS}
)

add_executable(asr5_control
        src/asr5_control.cpp
        src/websocket_endpoint.cpp
        src/ar5.cpp
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

