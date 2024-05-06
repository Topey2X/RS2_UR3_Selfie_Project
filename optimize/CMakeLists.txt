cmake_minimum_required(VERSION 3.0.2)
project(optimize)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Path.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(optimization src/main.cpp)
add_library(functions src/function.cpp)
target_link_libraries(optimization functions ${catkin_LIBRARIES})
