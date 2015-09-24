cmake_minimum_required(VERSION 2.8.3)
project(agent_test)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  std_msgs
  geometry_msgs
  message_generation
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
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
  DEPENDS
    Eigen
)

set(BIN_AGENT_TEST agent_test)

# Agent Test:
add_executable(${BIN_AGENT_TEST}
  src/agent_test.cpp
  src/agent_core.cpp
)
target_link_libraries(${BIN_AGENT_TEST}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_dependencies(${BIN_AGENT_TEST}
  ${catkin_EXPORTED_TARGETS}
)
