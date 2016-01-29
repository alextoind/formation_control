cmake_minimum_required(VERSION 2.8.3)
project(formation_control)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  std_msgs
  geometry_msgs
  visualization_msgs
  eigen_conversions
  message_generation
  tf
  interactive_markers
)
find_package(Eigen REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
    FormationStatistics.msg
    FormationStatisticsStamped.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

include_directories(include
  include/packet_manager
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    message_runtime
    tf
    interactive_markers
  DEPENDS
    Eigen
)

# Agent:
set(BIN_AGENT agent)

add_executable(${BIN_AGENT}
  src/agent_node.cpp
  src/agent_core.cpp
)
target_link_libraries(${BIN_AGENT}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_dependencies(${BIN_AGENT}
  ${catkin_EXPORTED_TARGETS}
)

# Visualization:
set(BIN_VISUALIZATION visualization)

add_executable(${BIN_VISUALIZATION}
  src/visualization_node.cpp
  src/visualization_core.cpp
)
target_link_libraries(${BIN_VISUALIZATION}
  ${catkin_LIBRARIES}
)
add_dependencies(${BIN_VISUALIZATION}
  ${catkin_EXPORTED_TARGETS}
)
