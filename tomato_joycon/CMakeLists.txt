cmake_minimum_required(VERSION 3.0.2)
project(tomato_joycon)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(joy_listener src/joy_listener.cpp)

target_link_libraries(joy_listener
    ${catkin_LIBRARIES}
    )
