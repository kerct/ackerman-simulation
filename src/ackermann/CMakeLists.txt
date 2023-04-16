cmake_minimum_required(VERSION 3.0.2)
project(ackermann)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(solver src/solver.cpp)
target_link_libraries(solver ${catkin_LIBRARIES})

add_executable(visualizer src/visualizer.cpp)
target_link_libraries(visualizer ${catkin_LIBRARIES})

add_executable(path_tracker src/path_tracker.cpp)
target_link_libraries(path_tracker ${catkin_LIBRARIES})
