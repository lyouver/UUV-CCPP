cmake_minimum_required(VERSION 3.0.2)
project(uuv_mpc_adapter)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O2 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  uuv_control_msgs
  trajectory_planner
  global_planner
  dynamic_predictor
  map_manager
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mpc_uuv_adapter_node src/mpc_uuv_adapter_node.cpp)
add_dependencies(mpc_uuv_adapter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_uuv_adapter_node ${catkin_LIBRARIES})
