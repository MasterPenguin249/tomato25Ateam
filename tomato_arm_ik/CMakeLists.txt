cmake_minimum_required(VERSION 3.0.2)
project(tomato_arm_ik)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  dynamixel_sdk
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm_ik src/arm_ik.cpp)
add_executable(arm_ik_joy src/arm_ik_joy.cpp)
add_executable(crane_control src/crane_control.cpp)

target_link_libraries(arm_ik
  ${catkin_LIBRARIES}
)
target_link_libraries(arm_ik_joy
  ${catkin_LIBRARIES}
)
target_link_libraries(crane_control
  ${catkin_LIBRARIES}
)
