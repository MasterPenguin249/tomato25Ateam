cmake_minimum_required(VERSION 3.0.2)
project(tomato_pubsub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)

target_link_libraries(talker
    ${catkin_LIBRARIES}
    )
target_link_libraries(listener
    ${catkin_LIBRARIES}
    )
