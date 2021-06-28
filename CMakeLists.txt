cmake_minimum_required(VERSION 2.8.3)
project(thinkgear_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp serial std_msgs
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(thinkgear_node src/thinkgear_node.cpp)

target_link_libraries(thinkgear_node
   ${catkin_LIBRARIES}
)
